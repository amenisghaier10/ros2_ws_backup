import asyncio
import websockets


async def listen():
     uri = "ws://localhost:8765"
     print(f"Connecting to {uri}...")
     async with websockets.connect(uri) as websocket:
         print("connected!")
         while True:
             message = await websocket.recv()
             print(f"Received from server: {message}")

if __name__ == "__main__":
    asyncio.run(listen())
