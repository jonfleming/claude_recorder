// Simple Node.js server to receive audio/wav POST and save to file
const express = require('express');
const fs = require('fs');
const path = require('path');
const { v4: uuidv4 } = require('uuid');

const app = express();
const port = 3000;
const uploadDir = path.join(__dirname, 'uploads');

// Ensure the uploads directory exists
if (!fs.existsSync(uploadDir)) {
  fs.mkdirSync(uploadDir);
}

app.get('/', (req, res) => {
  res.send('Audio upload server is running');
});

// Middleware to handle raw body for audio/wav
app.use('/upload', express.raw({ type: 'audio/wav', limit: '10mb' }));

app.post('/upload', (req, res) => {
  const fileId = uuidv4();
  const filename = path.join(uploadDir, `${fileId}.wav`);

  content = req.body;
  fs.writeFile(filename, content, (err) => {
    if (err) {
      console.error('Failed to save file:', err);
      return res.status(500).send('Error saving file');
    }
    console.log(`Saved audio to ${filename}`);
    res.status(200).send('Upload received');
  });
});

app.listen(port, () => {
  console.log(`Audio upload server running at http://localhost:${port}`);
});