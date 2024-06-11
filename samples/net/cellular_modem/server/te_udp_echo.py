# Copyright (c) 2023, Bjarki Arge Andreasen
# SPDX-License-Identifier: Apache-2.0

import socket
import threading
import select

class TEUDPEcho():
    def __init__(self):
        self.running = True
        self.on_terminated = None
        self.thread = threading.Thread(target=self._target_)

    def start(self):
        self.thread.start()

    def stop(self):
        if self.thread == threading.current_thread():
            return
        self.running = False
        self.thread.join(1)

    def signal_terminated(self, on_terminated):
        self.on_terminated = on_terminated

    def _target_(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.bind(('0.0.0.0', 7780))

        while self.running:
            try:
                ready_to_read, _, _ = select.select([sock], [], [], 0.5)

                if not ready_to_read:
                    continue

                data, address = sock.recvfrom(4096)
                print(f'udp echo {len(data)} bytes to  {address[0]}:{address[1]}')
                sock.sendto(data, address)

            except Exception as e:
                print(e)
                if self.on_terminated is not None:
                    self.on_terminated()
                break

        sock.close()
