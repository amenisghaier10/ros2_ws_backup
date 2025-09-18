import asyncio
import websockets

connected_clients = set()

async def handler(websocket):
     connected_clients.add(websocket)
     try: 
         async for message in websocket: 
             print(f"Received from client: {message}")
     finally:
         connected_clients.remove(websocket)

async def main():
     async with websockets.serve(handler, "0.0.0.0", 8765):
         print("WebSocket server started on ws://localhost:8765")
         await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
