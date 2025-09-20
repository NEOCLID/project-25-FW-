const pressureDisplay = document.getElementById('pressure-display');
const NUM_SENSORS = 6;

async function fetchData() {
    try {
        // Ensure you are using the correct IP address for your backend server
        const response = await fetch('http://localhost:8000/latest_seat_data');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();
        updateDisplay(data.pressure);
    } catch (error) {
        console.error('Error fetching data:', error);
        pressureDisplay.innerHTML = "<p>Could not fetch data from server. Is the backend running?</p>";
    }
}

function updateDisplay(pressureValues) {
    pressureDisplay.innerHTML = ''; // Clear previous display
    if (!pressureValues || pressureValues.length !== NUM_SENSORS) {
        pressureDisplay.innerHTML = "<p>Waiting for valid data...</p>";
        return;
    }

    for (let i = 0; i < NUM_SENSORS; i++) {
        const pressureValue = pressureValues[i];
        
        const wrapper = document.createElement('div');
        wrapper.className = 'sensor-wrapper';

        const label = document.createElement('div');
        label.className = 'sensor-label';
        label.textContent = `Sensor ${i + 1}`;

        const pressureBar = document.createElement('div');
        pressureBar.className = 'pressure-bar';
        pressureBar.textContent = pressureValue;
        
        // Map pressure value to a color (0-4095 range)
        // Blue (low pressure) to Red (high pressure)
        const colorValue = Math.min(255, Math.floor(pressureValue / 4095 * 255));
        pressureBar.style.backgroundColor = `rgb(${colorValue}, 0, ${255 - colorValue})`;
        
        wrapper.appendChild(label);
        wrapper.appendChild(pressureBar);
        pressureDisplay.appendChild(wrapper);
    }
}

// Fetch data every 2 seconds
setInterval(fetchData, 2000);

// Initial fetch
fetchData();