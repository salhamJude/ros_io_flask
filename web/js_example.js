
// Connect to the WebSocket server at the specified URL
const socket = io.connect("http://localhost:5000");

// Event listener for successful connection to the server
socket.on('connect', () => {
    console.log('Connected to Socket.IO server');
});

// Event listener for messages received from the server
socket.on('message', (data) => {
    console.log('Response from server:', data);
});

// Sending a test message to the server
socket.emit('message', { data: 'Hello from the client!' });

// Additional example of emitting a custom event
// Replace 'cameraState' and 'moveDirection' as needed based on your server's setup
socket.emit('cameraState', { state: 'ON' });
socket.emit('moveDirection', { direction: 'UP' });

// Handle disconnection
socket.on('disconnect', () => {
    console.log('Disconnected from Socket.IO server');
});
