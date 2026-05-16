"""
MeshOps event relay server.
Run with:  pip install fastapi uvicorn && python server.py

Receives POST /data/{uuid} from the backend script.
Serves  GET  /data/{uuid} for the frontend to poll.
"""

from collections import defaultdict
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Any
import uvicorn

app = FastAPI()

# Allow the frontend (localhost:3000) to talk to this server
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Store the last 50 events per UUID
event_store: dict[int, list[dict]] = defaultdict(list)
MAX_EVENTS = 50


@app.post("/data/{uuid}")
async def receive_event(uuid: int, body: dict):
    """Backend script POSTs parsed mesh events here."""
    event_store[uuid].append(body)
    # Keep only the latest MAX_EVENTS
    if len(event_store[uuid]) > MAX_EVENTS:
        event_store[uuid] = event_store[uuid][-MAX_EVENTS:]
    print(f"[UUID {uuid}] received: {body.get('packet')} from sender {body.get('sender')}")
    return {"status": "ok"}


@app.get("/data/{uuid}")
async def get_events(uuid: int):
    """Frontend polls here to get latest events."""
    events = event_store[uuid]
    # Return and clear — so each poll only gets new events
    event_store[uuid] = []
    return events


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
