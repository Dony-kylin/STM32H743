import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";
import { writeFile } from "node:fs/promises";

const sourcePath = process.argv[2];
const outputDir = process.argv[3];
if (!sourcePath || !outputDir) {
  throw new Error("Usage: analyze.mjs <xlsx> <output-dir>");
}

const workbook = await SpreadsheetFile.importXlsx(await FileBlob.load(sourcePath));
const overview = await workbook.inspect({
  kind: "workbook,sheet,table",
  include: "id,name",
  tableMaxRows: 1000,
  tableMaxCols: 30,
  maxChars: 20000,
});
console.log("=== OVERVIEW ===");
console.log(overview.ndjson);
const errorScan = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
  options: { useRegex: true, maxResults: 300 },
  summary: "final formula error scan",
});
console.log("=== FORMULA ERROR SCAN ===");
console.log(errorScan.ndjson);

for (let index = 0; index < 32; index += 1) {
  let sheet;
  try {
    sheet = workbook.worksheets.getItemAt(index);
  } catch {
    break;
  }
  if (!sheet) break;

  const used = sheet.getUsedRange(true);
  console.log(`=== SHEET ${index}: ${sheet.name} ===`);
  console.log(`ADDRESS=${used.address}`);
  const values = used.values;
  const valid = values
    .map((row, rowIndex) => ({ row: rowIndex + 1, x: Number(row[0]), y: Number(row[2]) }))
    .filter(({ x, y }) => Number.isFinite(x) && Number.isFinite(y) && x > 0 && y > 0);

  const n = valid.length;
  const sumX = valid.reduce((s, p) => s + p.x, 0);
  const sumY = valid.reduce((s, p) => s + p.y, 0);
  const meanX = sumX / n;
  const meanY = sumY / n;
  const sxx = valid.reduce((s, p) => s + (p.x - meanX) ** 2, 0);
  const sxy = valid.reduce((s, p) => s + (p.x - meanX) * (p.y - meanY), 0);
  const slope = sxy / sxx;
  const intercept = meanY - slope * meanX;
  const slopeOrigin = valid.reduce((s, p) => s + p.x * p.y, 0) /
    valid.reduce((s, p) => s + p.x * p.x, 0);
  const ssTot = valid.reduce((s, p) => s + (p.y - meanY) ** 2, 0);
  const ssRes = valid.reduce((s, p) => s + (p.y - (slope * p.x + intercept)) ** 2, 0);
  const errors = valid.map((p) => p.y - p.x);
  const ratios = valid.map((p) => p.y / p.x);
  const rmseIdentity = Math.sqrt(errors.reduce((s, e) => s + e * e, 0) / n);
  const maxAbsError = Math.max(...errors.map(Math.abs));
  const minError = Math.min(...errors);
  const maxError = Math.max(...errors);
  const meanRatio = ratios.reduce((s, r) => s + r, 0) / n;
  const effectiveGainFromDiv4 = 4 * slopeOrigin;
  const correctedErrors = valid.map((p) => p.y / slopeOrigin - p.x);
  const correctedRmse = Math.sqrt(correctedErrors.reduce((s, e) => s + e * e, 0) / n);
  const correctedMax = Math.max(...correctedErrors.map(Math.abs));
  const affineCorrectedErrors = valid.map((p) => (p.y - intercept) / slope - p.x);
  const affineCorrectedRmse = Math.sqrt(affineCorrectedErrors.reduce((s, e) => s + e * e, 0) / n);
  const affineCorrectedMax = Math.max(...affineCorrectedErrors.map(Math.abs));
  const result = {
    n,
    firstRow: valid[0].row,
    lastRow: valid[n - 1].row,
    xMin: valid[0].x,
    xMax: valid[n - 1].x,
    affineFit: { slope, intercept, r2: 1 - ssRes / ssTot },
    throughOriginSlope: slopeOrigin,
    measuredOverInputMean: meanRatio,
    rawError_mVpp: { min: minError, max: maxError, maxAbs: maxAbsError, rmse: rmseIdentity },
    impliedFrontendGainIfCurrentDivisorIs4: effectiveGainFromDiv4,
    afterOriginCorrection: { rmse_mVpp: correctedRmse, maxAbs_mVpp: correctedMax },
    afterAffineCorrection: { rmse_mVpp: affineCorrectedRmse, maxAbs_mVpp: affineCorrectedMax },
    samples: valid.filter((p) => [50, 100, 150, 200, 250].includes(p.x)),
  };
  console.log(`STATS=${JSON.stringify(result, null, 2)}`);

  const image = await workbook.render({
    sheetName: sheet.name,
    range: `A1:C${valid[n - 1].row}`,
    scale: 1.5,
    autoCrop: "all",
    format: "png",
  });
  const safeName = String(sheet.name).replace(/[\\/:*?"<>|]/g, "_");
  console.log(`RENDER_KEYS=${JSON.stringify(Object.keys(image ?? {}))}`);
  console.log(`RENDER_PROTO=${JSON.stringify(Object.getOwnPropertyNames(Object.getPrototypeOf(image ?? {})))}`);
  if (typeof image?.save === "function") {
    await image.save(`${outputDir}/${index}_${safeName}.png`);
  } else if (typeof image?.image?.save === "function") {
    await image.image.save(`${outputDir}/${index}_${safeName}.png`);
  } else if (typeof image?.arrayBuffer === "function") {
    await writeFile(`${outputDir}/${index}_${safeName}.png`, Buffer.from(await image.arrayBuffer()));
  } else {
    console.log(`RENDER_VALUE=${JSON.stringify(image)}`);
  }
}
