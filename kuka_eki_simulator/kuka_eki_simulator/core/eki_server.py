"""TCP motion channel and UDP meta channel servers (KRC side)."""

import logging
import socket

logger = logging.getLogger(__name__)

MOTION_FRAME_DELIMITER = b"</RobotCommand>"


class MotionServer:
    """Non-blocking TCP server for the EKI motion channel.

    Accepts a single client, collects complete <RobotCommand> frames from the
    incoming byte stream and re-accepts after the client disconnects.
    """

    def __init__(self, host, port):
        self.host = host
        self.port = int(port)
        self._listener = None
        self._conn = None
        self._buffer = b""

    def start(self):
        """Bind and start listening for the motion channel client."""
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((self.host, self.port))
        self._listener.listen(1)
        self._listener.setblocking(False)
        logger.info("Motion server listening on %s:%d", self.host, self.port)

    def receive(self):
        """Return the list of complete command frames received so far."""
        frames = []
        if self._conn is None:
            try:
                self._conn, address = self._listener.accept()
                self._conn.setblocking(False)
                logger.info("Motion channel client connected from %s", address)
            except BlockingIOError:
                return frames
        while True:
            try:
                data = self._conn.recv(4096)
            except BlockingIOError:
                break
            except OSError as error:
                logger.error("Motion channel receive error: %s", error)
                self._disconnect()
                break
            if not data:
                logger.info("Motion channel client disconnected")
                self._disconnect()
                break
            self._buffer += data
            parts = self._buffer.split(MOTION_FRAME_DELIMITER)
            self._buffer = parts.pop()
            frames.extend(part + MOTION_FRAME_DELIMITER for part in parts)
        return frames

    def send(self, data):
        """Send bytes to the connected client, if any."""
        if self._conn is None:
            return
        try:
            self._conn.sendall(data)
        except OSError as error:
            logger.error("Motion channel send error: %s", error)
            self._disconnect()

    def is_connected(self):
        """Return True if a client is connected."""
        return self._conn is not None

    def _disconnect(self):
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
        self._conn = None
        self._buffer = b""

    def close(self):
        """Close the connection and the listening socket."""
        self._disconnect()
        if self._listener is not None:
            self._listener.close()


class MetaServer:
    """Non-blocking UDP server for the EKI meta channel.

    The client registers itself by sending a first datagram (the driver sends
    a ";" ping right after connecting). State frames are only sent to the
    registered client address.
    """

    def __init__(self, host, port):
        self.host = host
        self.port = int(port)
        self._socket = None
        self._client = None

    def start(self):
        """Bind the UDP socket of the meta channel."""
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self.host, self.port))
        self._socket.setblocking(False)
        logger.info("Meta server listening on %s:%d", self.host, self.port)

    def register_client(self):
        """Consume any pending datagrams and remember the sender address."""
        if self._socket is None:
            return
        while True:
            try:
                _, address = self._socket.recvfrom(1024)
            except BlockingIOError:
                return
            except OSError as error:
                logger.error("Meta channel receive error: %s", error)
                return
            if self._client != address:
                logger.info("Meta channel client registered from %s", address)
            self._client = address

    def send(self, data):
        """Send bytes to the registered client, if any."""
        if self._socket is None or self._client is None:
            return
        try:
            self._socket.sendto(data, self._client)
        except OSError as error:
            logger.error("Meta channel send error: %s", error)

    def has_client(self):
        """Return True if a client has registered."""
        return self._client is not None

    def close(self):
        """Close the UDP socket."""
        if self._socket is not None:
            self._socket.close()
