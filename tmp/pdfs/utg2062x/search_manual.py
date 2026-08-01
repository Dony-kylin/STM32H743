from pathlib import Path
from pypdf import PdfReader

pdf_path = Path(__file__).with_name("manual.pdf")
reader = PdfReader(str(pdf_path))
terms = [
    "MATLAB", "USBTMC", "USB-TMC", "谐波", "HARM", "VISA",
    ":BASE:FREQuency", ":BASE:AMPLitude", ":BASE:OFFSet",
    "HARMonic:TOTal", "HARM:ORDER", "*IDN?",
]

for page_no, page in enumerate(reader.pages, start=1):
    text = page.extract_text() or ""
    matched = [term for term in terms if term.lower() in text.lower()]
    if matched:
        compact = " ".join(text.split())
        print(f"===== PDF_PAGE {page_no} MATCH {','.join(matched)} =====")
        print(compact[:5000])
