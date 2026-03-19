from pathlib import Path
from reportlab.lib.pagesizes import LETTER
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer
from reportlab.lib.styles import getSampleStyleSheet

base = Path(__file__).resolve().parent
src = base / "integration_readiness_irr_dev_report.txt"
out = base / "Integration_Readiness_irr-dev.pdf"

text = src.read_text(encoding="utf-8")
styles = getSampleStyleSheet()
normal = styles["Normal"]
normal.leading = 14
heading = styles["Heading2"]
heading.spaceBefore = 10
heading.spaceAfter = 6

doc = SimpleDocTemplate(str(out), pagesize=LETTER, leftMargin=54, rightMargin=54, topMargin=54, bottomMargin=54)
story = []

for line in text.splitlines():
    if not line.strip():
        story.append(Spacer(1, 8))
        continue
    if line.startswith("Software/Firmware Integration Readiness") or line.startswith("Senior Design B"):
        story.append(Paragraph(f"<b>{line}</b>", styles["Title"]))
        story.append(Spacer(1, 8))
    elif line[:2].isdigit() or (line and line[0].isdigit() and "." in line[:4]):
        story.append(Paragraph(f"<b>{line}</b>", heading))
    elif line.startswith("-"):
        story.append(Paragraph(f"• {line[1:].strip()}", normal))
    else:
        story.append(Paragraph(line, normal))

doc.build(story)
print(f"Generated: {out}")