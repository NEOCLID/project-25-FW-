import 'dotenv/config';
import fetch from 'node-fetch';

const token = process.env.FIGMA_TOKEN;
const fileKey = process.env.FIGMA_FILE_KEY;

const res = await fetch(`https://api.figma.com/v1/files/${fileKey}`, {
  headers: { 'X-Figma-Token': token }
});
if (!res.ok) throw new Error(`Figma API error ${res.status}`);
const fileJson = await res.json();
await Bun.write('figma-raw.json', JSON.stringify(fileJson, null, 2));
console.log('saved figma-raw.json');
