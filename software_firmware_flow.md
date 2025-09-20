```mermaid
graph TD
    subgraph User and Seat
        A[User sits on or interacts with the seat]
    end

    subgraph Firmware (ESP32)
        A --> F1{Read Pressure & Distance Sensors};
        F1 --> F2{Is pressure unbalanced?};
        F2 -- Yes --> F3{Is distance limit NOT reached?};
        F3 -- Yes --> F4[Activate motor to adjust seat];
        F4 --> F5[Send real-time data via Serial];
        F2 -- No --> F6[Keep motors idle];
        F3 -- No --> F6;
        F6 --> F5;
        F5 --> F1; 
    end

    subgraph Backend (Python/FastAPI Server)
        F5 --> B1[Listens for and reads Serial data from ESP32];
        B1 --> B2[Parses sensor data];
        B2 --> B3[Stores data in seat_data.db];
        B3 --> B4(API Endpoint e.g., /seat-data);
    end

    subgraph Frontend (Browser)
        U1[User opens index.html] --> U2[JavaScript requests data];
        U2 --> B4;
        B4 --> U3[Receives JSON data];
        U3 --> U4[Updates UI to visualize seat status];
        U4 --> U2; 
    end

    style F4 fill:#f9f,stroke:#333,stroke-width:2px
    style F6 fill:#ccf,stroke:#333,stroke-width:2px
```
