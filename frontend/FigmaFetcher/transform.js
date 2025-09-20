import fs from 'fs';

const raw = JSON.parse(fs.readFileSync('figma-raw.json','utf8'));
const doc = raw.document;
function walk(node, parentAutoLayout = null) {
  const base = {
    id: node.id, name: node.name, type: node.type,
    w: node.absoluteBoundingBox?.width, h: node.absoluteBoundingBox?.height,
    fills: node.fills, strokes: node.strokes, characters: node.characters
  };
  // Auto layout (Figma uses layoutMode: HORIZONTAL|VERTICAL, itemSpacing, padding, etc.)
  if (node.layoutMode) {
    base.layout = {
      mode: node.layoutMode,              // row/column
      spacing: node.itemSpacing ?? 0,
      padding: {
        l: node.paddingLeft ?? 0, r: node.paddingRight ?? 0,
        t: node.paddingTop ?? 0, b: node.paddingBottom ?? 0
      },
      alignItems: node.counterAxisAlignItems,
      justifyContent: node.primaryAxisAlignItems
    };
  }
  if (node.constraints) base.constraints = node.constraints;
  if (node.styles) base.styles = node.styles;      // references to text/color styles
  if (node.type === 'COMPONENT' || node.type === 'COMPONENT_SET') base.isComponent = true;
  if (node.children) base.children = node.children.map(c => walk(c, node.layoutMode || parentAutoLayout));
  return base;
}

const summary = { name: doc.name, pages: doc.children.map(walk) };
fs.writeFileSync('figma-semantic.json', JSON.stringify(summary, null, 2));
console.log('saved figma-semantic.json');
