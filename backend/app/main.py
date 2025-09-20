# ====== FastAPI: /ingest & retrieval (SQLite) ======
from datetime import datetime, timezone
from typing import List, Optional
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
import sqlite3

from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(title="Smart Seat Ingest")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

DB_FILE = "seat_data.db"

def init_db():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("""
        CREATE TABLE IF NOT EXISTS seat_measurements (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id TEXT NOT NULL,
            pressure_1 INTEGER, pressure_2 INTEGER, pressure_3 INTEGER,
            pressure_4 INTEGER, pressure_5 INTEGER, pressure_6 INTEGER,
            distance_1 INTEGER, distance_2 INTEGER,
            ts_server TEXT NOT NULL
        )
    """)
    conn.commit()
    conn.close()

init_db()

class SeatMeasurement(BaseModel):
    device_id: str = Field(..., min_length=1)
    pressure: List[int] = Field(..., min_items=6, max_items=6)
    distance: List[int] = Field(..., min_items=2, max_items=2)

@app.post("/ingest")
def ingest(m: SeatMeasurement):
    ts_server = datetime.now(timezone.utc).isoformat()

    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("""
        INSERT INTO seat_measurements (device_id, pressure_1, pressure_2, pressure_3, pressure_4, pressure_5, pressure_6, distance_1, distance_2, ts_server)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, (m.device_id, m.pressure[0], m.pressure[1], m.pressure[2], m.pressure[3], m.pressure[4], m.pressure[5], m.distance[0], m.distance[1], ts_server))
    conn.commit()
    conn.close()

    print(f"[INGEST] device_id={m.device_id}, pressure={m.pressure}, distance={m.distance}, ts_server={ts_server}")

    return {"status": "ok"}

@app.get("/latest_seat_data")
def latest_seat_data():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("""
        SELECT device_id, pressure_1, pressure_2, pressure_3, pressure_4, pressure_5, pressure_6, distance_1, distance_2, ts_server
        FROM seat_measurements
        ORDER BY id DESC
        LIMIT 1
    """)
    row = c.fetchone()
    conn.close()

    if row:
        return {
            "device_id": row[0],
            "pressure": [row[1], row[2], row[3], row[4], row[5], row[6]],
            "distance": [row[7], row[8]],
            "ts_server": row[9]
        }
    raise HTTPException(404, "No data available")