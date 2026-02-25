"""Analyze images embedded in the PDF to find compression targets."""
import fitz
import os

pdf_path = "data/VentCon_User_Manual.pdf"
doc = fitz.open(pdf_path)
fsize = os.path.getsize(pdf_path)
print(f"Pages: {len(doc)}")
print(f"File size: {fsize:,} bytes ({fsize/1024:.0f} KB)")
print()

total_img_size = 0
seen_xrefs = set()

for i, page in enumerate(doc):
    imgs = page.get_images(full=True)
    print(f"Page {i+1}: {len(imgs)} images")
    for j, img in enumerate(imgs):
        xref = img[0]
        if xref in seen_xrefs:
            print(f"  img[{j}] xref={xref}: (duplicate, already counted)")
            continue
        seen_xrefs.add(xref)
        base = doc.extract_image(xref)
        if base:
            raw = base["image"]
            w = base["width"]
            h = base["height"]
            ext = base["ext"]
            cs = base.get("colorspace", 0)
            print(f"  img[{j}] xref={xref}: {w}x{h}, ext={ext}, cs={cs}, size={len(raw):,}B")
            total_img_size += len(raw)
        else:
            print(f"  img[{j}] xref={xref}: could not extract")

print(f"\nTotal unique image data: {total_img_size:,} bytes ({total_img_size/1024:.0f} KB)")
print(f"Image share of file: {total_img_size/fsize*100:.1f}%")
print(f"Non-image data: {(fsize - total_img_size):,} bytes ({(fsize - total_img_size)/1024:.0f} KB)")
doc.close()
