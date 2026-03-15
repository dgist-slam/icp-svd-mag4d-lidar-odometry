/**
 * Data Loader — loads viz_data.json only
 * ICP is now computed in the browser via src/lib/icp.js
 */
export async function loadVizData() {
  const res = await fetch('/viz_data.json');
  return res.json();
}
