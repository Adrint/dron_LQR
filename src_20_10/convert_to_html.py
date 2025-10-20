"""
Konwersja Markdown do HTML (można wydrukować jako PDF)
"""

import markdown
import os

def md_to_html():
    """Konwertuje Markdown do HTML"""
    
    input_file = "DOKUMENTACJA_KOMPLETNA.md"
    output_file = "DOKUMENTACJA_KOMPLETNA.html"
    
    # Wczytaj markdown
    with open(input_file, 'r', encoding='utf-8') as f:
        md_content = f.read()
    
    # Konwertuj do HTML
    html_body = markdown.markdown(md_content, extensions=['tables', 'fenced_code', 'codehilite'])
    
    # Dodaj style CSS
    html = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Dokumentacja LQR Drona</title>
    <style>
        body {{
            font-family: 'Segoe UI', Arial, sans-serif;
            max-width: 900px;
            margin: 40px auto;
            padding: 20px;
            line-height: 1.6;
            color: #333;
        }}
        h1 {{
            color: #2c3e50;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
        }}
        h2 {{
            color: #34495e;
            border-bottom: 2px solid #95a5a6;
            padding-bottom: 8px;
            margin-top: 30px;
        }}
        h3 {{
            color: #555;
            margin-top: 25px;
        }}
        code {{
            background: #f4f4f4;
            padding: 2px 6px;
            border-radius: 3px;
            font-family: 'Consolas', monospace;
            font-size: 0.9em;
        }}
        pre {{
            background: #2d2d2d;
            color: #f8f8f2;
            padding: 15px;
            border-radius: 5px;
            overflow-x: auto;
        }}
        pre code {{
            background: none;
            padding: 0;
            color: #f8f8f2;
        }}
        table {{
            border-collapse: collapse;
            width: 100%;
            margin: 20px 0;
        }}
        th, td {{
            border: 1px solid #ddd;
            padding: 12px;
            text-align: left;
        }}
        th {{
            background: #3498db;
            color: white;
        }}
        tr:nth-child(even) {{
            background: #f9f9f9;
        }}
        blockquote {{
            border-left: 4px solid #3498db;
            padding-left: 20px;
            color: #555;
            font-style: italic;
            margin: 20px 0;
        }}
        .math {{
            font-family: 'Times New Roman', serif;
            font-style: italic;
        }}
        @media print {{
            body {{
                max-width: 100%;
                margin: 0;
            }}
            h1 {{
                page-break-before: always;
            }}
            h1:first-child {{
                page-break-before: avoid;
            }}
        }}
    </style>
</head>
<body>
{html_body}
</body>
</html>
"""
    
    # Zapisz HTML
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(html)
    
    print(f"✅ Utworzono {output_file}")
    print(f"\n📖 Jak wydrukować do PDF:")
    print(f"1. Otwórz plik w przeglądarce: {os.path.abspath(output_file)}")
    print(f"2. Ctrl+P (Drukuj)")
    print(f"3. Wybierz 'Zapisz jako PDF'")
    print(f"4. Zapisz plik")
    
    # Otwórz w przeglądarce
    os.system(f'start "" "{output_file}"')

if __name__ == "__main__":
    try:
        import markdown
    except ImportError:
        print("❌ Brak modułu markdown. Instaluję...")
        os.system("pip install markdown")
        import markdown
    
    md_to_html()
