import fs from "node:fs/promises";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const outputDir = process.argv[2];
if (!outputDir) throw new Error("Usage: build.mjs <output-dir>");
await fs.mkdir(outputDir, { recursive: true });

const workbook = Workbook.create();
const constraints = workbook.worksheets.add("题目约束");
const casesSheet = workbook.worksheets.add("测试用例");
const recordSheet = workbook.worksheets.add("测试记录");

const COLORS = {
  navy: "#163A5F",
  blue: "#245B8A",
  cyan: "#DDEBF7",
  paleBlue: "#EAF3F8",
  input: "#FFF2CC",
  green: "#E2F0D9",
  red: "#FCE4D6",
  gray: "#F2F2F2",
  text: "#1F2937",
  white: "#FFFFFF",
  border: "#D0D7DE",
};

let rngState = 20260801 >>> 0;
function rand() {
  rngState = (Math.imul(rngState, 1664525) + 1013904223) >>> 0;
  return rngState / 4294967296;
}
function roundHalf(value) {
  return Math.round(value * 2) / 2;
}
function shapeFactor(orders, ratios, phasesDeg) {
  const sampleCount = 65536;
  let lo = Number.POSITIVE_INFINITY;
  let hi = Number.NEGATIVE_INFINITY;
  for (let i = 0; i < sampleCount; i += 1) {
    const theta = 2 * Math.PI * i / sampleCount;
    let value = Math.sin(theta);
    for (let h = 0; h < orders.length; h += 1) {
      value += ratios[h] * Math.sin(orders[h] * theta + phasesDeg[h] * Math.PI / 180);
    }
    if (value < lo) lo = value;
    if (value > hi) hi = value;
  }
  return hi - lo;
}

const taskConfig = {
  1: { count: 34, signal: "ua", vppMin: 100, vppMax: 250, fMax: 200 },
  2: { count: 33, signal: "ub", vppMin: 50, vppMax: 250, fMax: 500 },
  3: { count: 33, signal: "ub+uJ", vppMin: 50, vppMax: 250, fMax: 500 },
};
const targetSets = {
  1: [100, 110, 125, 150, 175, 200, 225, 240, 250],
  2: [50, 60, 75, 100, 125, 150, 175, 200, 225, 240, 250],
  3: [50, 60, 75, 100, 125, 150, 175, 200, 225, 240, 250],
};
const harmonicCombos = [[2], [3], [4], [5], [2, 3], [2, 4], [3, 4], [3, 5], [4, 5]];
const oneRatios = [[0.25], [0.4], [0.6], [0.8], [1.0]];
const twoRatios = [[0.25, 0.15], [0.4, 0.2], [0.5, 0.3], [0.75, 0.4], [1.0, 0.5]];
const phase2Set = [0, 45, 90, 135, 180, 270];
const phase3Set = [0, 30, 60, 120, 180, 240];
const interferenceMHz = [1.0, 1.05, 1.2, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 7.0, 7.5, 8.2];

const overrides = {
  1: [
    { target: 100, f1: 10, orders: [2, 3], ratios: [0.5, 0.25], phases: [0, 0], tag: "幅值/频率下限" },
    { target: 250, f1: 10.5, orders: [3, 4], ratios: [0.5, 0.25], phases: [0, 0], tag: "幅值上限/题目示例型" },
    { target: 250, f1: 100, orders: [2], ratios: [0.4], phases: [0], tag: "200kHz频率上限" },
    { target: 150, f1: 50, orders: [3, 4], ratios: [0.4, 0.2], phases: [45, 120], tag: "200kHz边界/相位" },
  ],
  2: [
    { target: 50, f1: 10, orders: [2], ratios: [0.5], phases: [0], tag: "幅值/频率下限" },
    { target: 250, f1: 250, orders: [2], ratios: [0.4], phases: [0], tag: "幅值/500kHz上限" },
    { target: 150, f1: 125, orders: [3, 4], ratios: [0.4, 0.2], phases: [0, 0], tag: "500kHz频率上限" },
    { target: 100, f1: 100, orders: [4, 5], ratios: [0.5, 0.25], phases: [90, 180], tag: "高次谐波/500kHz" },
  ],
  3: [
    { target: 50, f1: 10, orders: [2, 3], ratios: [0.5, 0.25], phases: [0, 0], fj: 1.0, tag: "幅值/频率/干扰下限" },
    { target: 250, f1: 250, orders: [2], ratios: [0.4], phases: [0], fj: 1.05, tag: "幅值/500kHz上限" },
    { target: 150, f1: 125, orders: [3, 4], ratios: [0.4, 0.2], phases: [45, 120], fj: 3.5, tag: "500kHz边界/抗干扰" },
    { target: 100, f1: 100, orders: [4, 5], ratios: [0.5, 0.25], phases: [90, 180], fj: 7.5, tag: "混叠压力/500kHz" },
  ],
};

function makeGeneric(task, localIndex) {
  const cfg = taskConfig[task];
  const combo = harmonicCombos[(localIndex + task * 2) % harmonicCombos.length];
  const ratiosBase = combo.length === 1
    ? oneRatios[(localIndex + task) % oneRatios.length]
    : twoRatios[(localIndex + task) % twoRatios.length];
  const ratios = ratiosBase.map((r) => Math.round((r + (rand() - 0.5) * 0.04) * 1000) / 1000);
  const maxOrder = Math.max(...combo);
  const maxBase = cfg.fMax / maxOrder;
  const frequencyFraction = [0, 0.2, 0.35, 0.5, 0.65, 0.8, 0.95, 1][localIndex % 8];
  const f1 = frequencyFraction === 0
    ? 10
    : roundHalf(10 + (maxBase - 10) * frequencyFraction);
  const target = targetSets[task][localIndex % targetSets[task].length];
  const phases = combo.length === 1
    ? [phase2Set[localIndex % phase2Set.length]]
    : [phase2Set[localIndex % phase2Set.length], phase3Set[(localIndex + task) % phase3Set.length]];
  const topFrequency = f1 * maxOrder;
  let tag = "典型组合";
  if (target === cfg.vppMin || target === cfg.vppMax) tag = "幅值边界";
  if (f1 === 10) tag = `${tag}/频率下限`;
  if (Math.abs(topFrequency - cfg.fMax) <= 0.26) tag = `${tag}/频率上限`;
  if (Math.min(...ratios) <= 0.25) tag = `${tag}/弱谐波`;
  const fj = task === 3 ? interferenceMHz[localIndex % interferenceMHz.length] : null;
  if (task === 3 && fj > 4) tag = `${tag}/混叠压力`;
  return { target, f1, orders: combo, ratios, phases, fj, tag };
}

const cases = [];
let id = 1;
for (const task of [1, 2, 3]) {
  const cfg = taskConfig[task];
  for (let localIndex = 0; localIndex < cfg.count; localIndex += 1) {
    const spec = localIndex < overrides[task].length
      ? overrides[task][localIndex]
      : makeGeneric(task, localIndex);
    const factor = shapeFactor(spec.orders, spec.ratios, spec.phases);
    const maxComponent = spec.f1 * Math.max(...spec.orders);
    if (spec.target < cfg.vppMin || spec.target > cfg.vppMax) throw new Error(`Vpp out of range: ${id}`);
    if (spec.f1 < 10 || maxComponent > cfg.fMax + 1e-6) throw new Error(`Frequency out of range: ${id}`);
    cases.push({
      id,
      task,
      signal: cfg.signal,
      tag: spec.tag,
      target: spec.target,
      f1: spec.f1,
      count: spec.orders.length,
      order2: spec.orders[0],
      ratio2: spec.ratios[0],
      phase2: spec.phases[0],
      order3: spec.orders[1] ?? null,
      ratio3: spec.ratios[1] ?? null,
      phase3: spec.phases[1] ?? null,
      factor,
      interferenceVpp: task === 3 ? 200 : 0,
      interferenceMHz: task === 3 ? (spec.fj ?? interferenceMHz[localIndex % interferenceMHz.length]) : null,
      note: task === 3 && (spec.fj ?? 0) > 4
        ? "8MSPS下需关注干扰混叠；用于压力测试"
        : "基波初相位0°；幅值按总Vpp归一化",
    });
    id += 1;
  }
}
if (cases.length !== 100) throw new Error(`Expected 100 cases, got ${cases.length}`);

// Sheet 1: constraints and protocol.
constraints.showGridLines = false;
constraints.getRange("A1:H1").format = { fill: COLORS.navy, font: { bold: true, color: COLORS.white, size: 16 } };
constraints.getRange("A1").values = [["G题周期信号测量分析装置 - 100组测试计划"]];
constraints.getRange("A2").values = [["来源：G题_周期信号测量分析装置.pdf（第1-3页）；生成种子：20260801"]];
constraints.getRange("A4:H4").values = [["题目项", "被测信号Vpp", "分量频率范围", "单频干扰", "测试组数", "Vpp/RMS/谱幅误差", "基频误差", "完成时间"]];
constraints.getRange("A5:D7").values = [
  [1, "100-250 mVpp", "10-200 kHz", "无"],
  [2, "50-250 mVpp", "10-500 kHz", "无"],
  [3, "50-250 mVpp", "10-500 kHz", "200 mVpp，fJ≥1 MHz"],
];
constraints.getRange("E5:E7").formulas = [
  ["=COUNTIF('测试用例'!$B$5:$B$104,A5)"],
  ["=COUNTIF('测试用例'!$B$5:$B$104,A6)"],
  ["=COUNTIF('测试用例'!$B$5:$B$104,A7)"],
];
constraints.getRange("F5:H7").values = [
  ["≤5 mV", "≤1 kHz", "≤2000 ms"],
  ["≤5 mV", "≤1 kHz", "≤2000 ms"],
  ["≤5 mV", "≤1 kHz", "≤2000 ms"],
];
constraints.getRange("A9:B15").values = [
  ["通用约束", "被测周期信号由基波与1个或2个谐波分量组成。"],
  ["有效值", "理论RMS为真有效值；不同整数次谐波的平方和开方。"],
  ["频谱幅值", "基波/谐波A/谐波B列均为正弦分量峰值（Vpk），与题目频谱幅值定义一致。"],
  ["信号源设置", "若谐波发生器按总Vpp归一化：设置目标总Vpp并按相对幅值、相位设置谐波。"],
  ["独立分量模式", "若信号源可独立设置分量：使用测试用例中的基波/谐波A/谐波B分量Vpp。"],
  ["形状因子", "按65536点/基波周期扫相计算；H1峰值=目标Vpp/形状Vpp因子。"],
  ["压力点", "第3项含fJ>4 MHz数据，用于检查当前8MSPS系统的混叠抑制，不代表可忽略模拟滤波。"],
];
constraints.getRange("A4:H7").format.borders = { preset: "all", style: "thin", color: COLORS.border };
constraints.getRange("A4:H4").format = { fill: COLORS.blue, font: { bold: true, color: COLORS.white }, horizontalAlignment: "center" };
constraints.getRange("A5:H7").format = { fill: COLORS.paleBlue, verticalAlignment: "center" };
constraints.getRange("A9:A15").format = { fill: COLORS.cyan, font: { bold: true, color: COLORS.text } };
constraints.getRange("A9:B15").format.borders = { preset: "outside", style: "thin", color: COLORS.border };
constraints.getRange("B9:B15").format.wrapText = true;
constraints.getRange("A1:H15").format.font.name = "Microsoft YaHei";
constraints.getRange("A1:H15").format.rowHeight = 22;
constraints.getRange("A1:H1").format.rowHeight = 32;
constraints.getRange("A1:A15").format.columnWidth = 18;
constraints.getRange("B1:B15").format.columnWidth = 76;
constraints.getRange("C1:C15").format.columnWidth = 18;
constraints.getRange("D1:D15").format.columnWidth = 26;
constraints.getRange("E1:H15").format.columnWidth = 18;
constraints.getRange("A9:B15").format.rowHeight = 40;
constraints.getRange("A15:B15").format.rowHeight = 48;
constraints.freezePanes.freezeRows(4);

// Sheet 2: generated test cases.
const caseHeaders = [
  "编号", "题目项", "信号", "覆盖类型", "目标总Vpp_mV", "基频_kHz", "谐波数",
  "谐波A阶次", "谐波A频率_kHz", "谐波A/H1", "谐波A相位_deg",
  "谐波B阶次", "谐波B频率_kHz", "谐波B/H1", "谐波B相位_deg", "形状Vpp因子",
  "基波峰值_mV", "基波分量Vpp_mV", "谐波A峰值_mV", "谐波A分量Vpp_mV",
  "谐波B峰值_mV", "谐波B分量Vpp_mV", "理论RMS_mV", "干扰Vpp_mV", "干扰频率_MHz", "备注",
];
casesSheet.showGridLines = false;
casesSheet.getRange("A1:Z1").format = { fill: COLORS.navy, font: { bold: true, color: COLORS.white, size: 15 } };
casesSheet.getRange("A1").values = [["G题100组输入测试用例"]];
casesSheet.getRange("A2").values = [["蓝色列为信号源输入条件；浅蓝色列为公式计算的标称值。所有数据采用确定性生成，可重复使用。"]];
casesSheet.getRange("A4:Z4").values = [caseHeaders];
const caseValues = cases.map((c) => [
  c.id, c.task, c.signal, c.tag, c.target, c.f1, c.count,
  c.order2, null, c.ratio2, c.phase2,
  c.order3, null, c.ratio3, c.phase3, c.factor,
  null, null, null, null, null, null, null,
  c.interferenceVpp, c.interferenceMHz, c.note,
]);
casesSheet.getRange("A5:Z104").values = caseValues;
const formulaI = [];
const formulaM = [];
const formulaQW = [];
for (let row = 5; row <= 104; row += 1) {
  formulaI.push([`=F${row}*H${row}`]);
  formulaM.push([`=IF(L${row}="","",F${row}*L${row})`]);
  formulaQW.push([
    `=E${row}/P${row}`,
    `=2*Q${row}`,
    `=Q${row}*J${row}`,
    `=2*S${row}`,
    `=IF(G${row}=2,Q${row}*N${row},"")`,
    `=IF(U${row}="","",2*U${row})`,
    `=SQRT((Q${row}^2+S${row}^2+IF(U${row}="",0,U${row}^2))/2)`,
  ]);
}
casesSheet.getRange("I5:I104").formulas = formulaI;
casesSheet.getRange("M5:M104").formulas = formulaM;
casesSheet.getRange("Q5:W104").formulas = formulaQW;
casesSheet.getRange("A4:Z4").format = { fill: COLORS.blue, font: { bold: true, color: COLORS.white }, wrapText: true, horizontalAlignment: "center", verticalAlignment: "center" };
casesSheet.getRange("A5:P104").format.fill = COLORS.input;
casesSheet.getRange("Q5:W104").format.fill = COLORS.paleBlue;
casesSheet.getRange("X5:Z104").format.fill = COLORS.input;
casesSheet.getRange("A4:Z104").format.font.name = "Microsoft YaHei";
casesSheet.getRange("A4:Z104").format.borders = { preset: "inside", style: "thin", color: COLORS.border };
casesSheet.getRange("E5:F104").format.numberFormat = "0.0";
casesSheet.getRange("I5:I104").format.numberFormat = "0.0";
casesSheet.getRange("J5:J104").format.numberFormat = "0.0%";
casesSheet.getRange("M5:M104").format.numberFormat = "0.0";
casesSheet.getRange("N5:N104").format.numberFormat = "0.0%";
casesSheet.getRange("P5:P104").format.numberFormat = "0.000000";
casesSheet.getRange("Q5:W104").format.numberFormat = "0.000";
casesSheet.getRange("X5:Y104").format.numberFormat = "0.00";
casesSheet.getRange("A5:C104").format.horizontalAlignment = "center";
casesSheet.getRange("E5:Y104").format.horizontalAlignment = "right";
casesSheet.getRange("D5:D104").format.wrapText = true;
casesSheet.getRange("Z5:Z104").format.wrapText = true;
casesSheet.getRange("A1:A104").format.columnWidth = 10;
casesSheet.getRange("B1:C104").format.columnWidth = 11;
casesSheet.getRange("D1:D104").format.columnWidth = 24;
casesSheet.getRange("E1:W104").format.columnWidth = 15;
casesSheet.getRange("X1:Y104").format.columnWidth = 16;
casesSheet.getRange("Z1:Z104").format.columnWidth = 38;
casesSheet.getRange("A4:Z4").format.rowHeight = 38;
casesSheet.freezePanes.freezeRows(4);
casesSheet.freezePanes.freezeColumns(4);
const casesTable = casesSheet.tables.add("A4:Z104", true, "GTestCases");
casesTable.style = "TableStyleMedium2";
casesTable.showBandedRows = false;

// Sheet 3: record measured results and automatic pass/fail checks.
const recordHeaders = [
  "编号", "题目项", "目标Vpp_mV", "理论RMS_mV", "基频_kHz", "基波峰值_mV", "谐波A峰值_mV", "谐波B峰值_mV",
  "实测Vpp_mV", "Vpp误差_mV", "实测RMS_mV", "RMS误差_mV", "实测基频_kHz", "频率误差_kHz",
  "实测基波_mV", "基波误差_mV", "实测谐波A_mV", "谐波A误差_mV", "实测谐波B_mV", "谐波B误差_mV",
  "完成时间_ms", "Vpp判定", "RMS判定", "频率判定", "谱幅判定", "时间判定", "综合判定",
];
recordSheet.showGridLines = false;
recordSheet.getRange("A1:AA1").format = { fill: COLORS.navy, font: { bold: true, color: COLORS.white, size: 15 } };
recordSheet.getRange("A1").values = [["G题100组测试结果记录"]];
recordSheet.getRange("A2").values = [["黄色列填写装置实测值；误差和判定自动计算。频谱分量幅值使用峰值mV。"]];
recordSheet.getRange("A4:AA4").values = [recordHeaders];
const referenceFormulas = [];
const resultFormulas = [];
for (let row = 5; row <= 104; row += 1) {
  referenceFormulas.push([
    `='测试用例'!A${row}`,
    `='测试用例'!B${row}`,
    `='测试用例'!E${row}`,
    `='测试用例'!W${row}`,
    `='测试用例'!F${row}`,
    `='测试用例'!Q${row}`,
    `='测试用例'!S${row}`,
    `='测试用例'!U${row}`,
  ]);
  resultFormulas.push([
    `=IF(I${row}="","",I${row}-C${row})`,
    `=IF(K${row}="","",K${row}-D${row})`,
    `=IF(M${row}="","",M${row}-E${row})`,
    `=IF(O${row}="","",O${row}-F${row})`,
    `=IF(Q${row}="","",Q${row}-G${row})`,
    `=IF(OR(H${row}="",S${row}=""),"",S${row}-H${row})`,
    `=IF(I${row}="","",IF(ABS(J${row})<=5,"PASS","FAIL"))`,
    `=IF(K${row}="","",IF(ABS(L${row})<=5,"PASS","FAIL"))`,
    `=IF(M${row}="","",IF(ABS(N${row})<=1,"PASS","FAIL"))`,
    `=IF(OR(O${row}="",Q${row}="",IF(H${row}="",FALSE,S${row}="")),"",IF(AND(ABS(P${row})<=5,ABS(R${row})<=5,IF(H${row}="",TRUE,ABS(T${row})<=5)),"PASS","FAIL"))`,
    `=IF(U${row}="","",IF(U${row}<=2000,"PASS","FAIL"))`,
    `=IF(COUNTBLANK(V${row}:Z${row})>0,"",IF(COUNTIF(V${row}:Z${row},"FAIL")=0,"PASS","FAIL"))`,
  ]);
}
recordSheet.getRange("A5:H104").formulas = referenceFormulas;
recordSheet.getRange("J5:J104").formulas = resultFormulas.map((r) => [r[0]]);
recordSheet.getRange("L5:L104").formulas = resultFormulas.map((r) => [r[1]]);
recordSheet.getRange("N5:N104").formulas = resultFormulas.map((r) => [r[2]]);
recordSheet.getRange("P5:P104").formulas = resultFormulas.map((r) => [r[3]]);
recordSheet.getRange("R5:R104").formulas = resultFormulas.map((r) => [r[4]]);
recordSheet.getRange("T5:T104").formulas = resultFormulas.map((r) => [r[5]]);
recordSheet.getRange("V5:AA104").formulas = resultFormulas.map((r) => r.slice(6));
recordSheet.getRange("A4:AA4").format = { fill: COLORS.blue, font: { bold: true, color: COLORS.white }, wrapText: true, horizontalAlignment: "center", verticalAlignment: "center" };
recordSheet.getRange("A5:H104").format.fill = COLORS.paleBlue;
for (const inputColumn of ["I", "K", "M", "O", "Q", "S", "U"]) {
  recordSheet.getRange(`${inputColumn}5:${inputColumn}104`).format.fill = COLORS.input;
}
for (const calcColumn of ["J", "L", "N", "P", "R", "T", "V", "W", "X", "Y", "Z", "AA"]) {
  recordSheet.getRange(`${calcColumn}5:${calcColumn}104`).format.fill = COLORS.gray;
}
recordSheet.getRange("A4:AA104").format.font.name = "Microsoft YaHei";
recordSheet.getRange("A4:AA104").format.borders = { preset: "inside", style: "thin", color: COLORS.border };
recordSheet.getRange("C5:T104").format.numberFormat = "0.000";
recordSheet.getRange("U5:U104").format.numberFormat = "0";
recordSheet.getRange("A5:B104").format.horizontalAlignment = "center";
recordSheet.getRange("V5:AA104").format.horizontalAlignment = "center";
recordSheet.getRange("A1:B104").format.columnWidth = 10;
recordSheet.getRange("C1:U104").format.columnWidth = 15;
recordSheet.getRange("V1:AA104").format.columnWidth = 12;
recordSheet.getRange("A4:AA4").format.rowHeight = 38;
recordSheet.freezePanes.freezeRows(4);
recordSheet.freezePanes.freezeColumns(2);
const recordTable = recordSheet.tables.add("A4:AA104", true, "GTestRecords");
recordTable.style = "TableStyleMedium2";
recordTable.showBandedRows = false;
recordSheet.getRange("V5:AA104").conditionalFormats.add("containsText", { text: "PASS", format: { fill: COLORS.green, font: { color: "#006100", bold: true } } });
recordSheet.getRange("V5:AA104").conditionalFormats.add("containsText", { text: "FAIL", format: { fill: COLORS.red, font: { color: "#9C0006", bold: true } } });

const keyInspect = await workbook.inspect({
  kind: "table",
  range: "测试用例!A4:Z12",
  include: "values,formulas",
  tableMaxRows: 12,
  tableMaxCols: 26,
  maxChars: 12000,
});
console.log("=== KEY RANGE ===");
console.log(keyInspect.ndjson);
const errorScan = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
  options: { useRegex: true, maxResults: 300 },
  summary: "final formula error scan",
});
console.log("=== FORMULA ERRORS ===");
console.log(errorScan.ndjson);

for (const [sheetName, range, fileName, scale] of [
  ["题目约束", "A1:H16", "preview_constraints.png", 1.2],
  ["测试用例", "A1:Z104", "preview_cases.png", 0.75],
  ["测试记录", "A1:AA104", "preview_records.png", 0.75],
]) {
  const preview = await workbook.render({ sheetName, range, scale, format: "png" });
  await fs.writeFile(`${process.cwd()}/${fileName}`, new Uint8Array(await preview.arrayBuffer()));
}

const output = await SpreadsheetFile.exportXlsx(workbook);
await output.save(`${outputDir}/G题_100组测试数据.xlsx`);
console.log(`OUTPUT=${outputDir}/G题_100组测试数据.xlsx`);
