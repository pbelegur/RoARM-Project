import asyncio, websockets, datetime

clients = set()

async def handler(ws):
    ip = ws.remote_address[0]
    print(f"[+] {ip} connected")
    clients.add(ws)
    try:
        async for msg in ws:
            if isinstance(msg, (bytes, bytearray)):
                # forward binary frames to browsers
                still_open = set()
                for c in clients.copy():
                    if c is ws:
                        continue
                    try:
                        if getattr(c, "closed", False) or getattr(c, "state", None) == "CLOSED":
                            continue
                        await c.send(msg)
                        still_open.add(c)
                    except Exception as e:
                        print(f"[!] send failed to {getattr(c,'remote_address','?')}: {e}")
                clients.clear()
                clients.update(still_open)
                print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] Frame {len(msg)} bytes → {len(still_open)} browser(s)")
    except websockets.exceptions.ConnectionClosedOK:
        print(f"[-] {ip} closed connection normally")
    except websockets.exceptions.ConnectionClosedError:
        print(f"[x] {ip} connection error")
    finally:
        clients.discard(ws)

async def main():
    print("[*] WebSocket server running on ws://0.0.0.0:8765")
    async with websockets.serve(
        handler,
        "0.0.0.0",
        8765,
        max_size=2**22,
        ping_interval=None,   # disable ping timeouts
        ping_timeout=None
    ):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
