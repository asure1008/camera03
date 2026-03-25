#!/usr/bin/env python3
import argparse
import asyncio
import signal
import ssl


async def _pipe(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    try:
        while True:
            chunk = await reader.read(65536)
            if not chunk:
                break
            writer.write(chunk)
            await writer.drain()
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass


async def _handle_client(
    client_reader: asyncio.StreamReader,
    client_writer: asyncio.StreamWriter,
    upstream_host: str,
    upstream_port: int,
) -> None:
    try:
        upstream_reader, upstream_writer = await asyncio.open_connection(
            upstream_host, upstream_port
        )
    except Exception:
        try:
            client_writer.close()
            await client_writer.wait_closed()
        except Exception:
            pass
        return

    await asyncio.gather(
        _pipe(client_reader, upstream_writer),
        _pipe(upstream_reader, client_writer),
        return_exceptions=True,
    )


async def _main() -> None:
    ap = argparse.ArgumentParser(description="TLS TCP proxy for Camera03")
    ap.add_argument("--listen-host", default="0.0.0.0")
    ap.add_argument("--listen-port", type=int, default=8443)
    ap.add_argument("--upstream-host", default="127.0.0.1")
    ap.add_argument("--upstream-port", type=int, default=8080)
    ap.add_argument("--cert", required=True)
    ap.add_argument("--key", required=True)
    args = ap.parse_args()

    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(certfile=args.cert, keyfile=args.key)

    server = await asyncio.start_server(
        lambda r, w: _handle_client(r, w, args.upstream_host, args.upstream_port),
        host=args.listen_host,
        port=args.listen_port,
        ssl=ssl_ctx,
    )

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _stop() -> None:
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _stop)

    async with server:
        await stop_event.wait()


if __name__ == "__main__":
    asyncio.run(_main())
