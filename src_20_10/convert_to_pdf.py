"""
Skrypt do konwersji dokumentacji Markdown do PDF
Wymaga: pip install markdown pdfkit
"""

import os
import sys

def convert_to_pdf():
    """
    Konwertuje dokumentację Markdown do PDF używając pandoc
    """
    
    input_file = "DOKUMENTACJA_KOMPLETNA.md"
    output_file = "DOKUMENTACJA_KOMPLETNA.pdf"
    
    if not os.path.exists(input_file):
        print(f"❌ Błąd: Nie znaleziono pliku {input_file}")
        return False
    
    print(f"📄 Konwersja {input_file} → {output_file}")
    
    # Sprawdź czy pandoc jest zainstalowany
    pandoc_check = os.system("pandoc --version > nul 2>&1")
    
    if pandoc_check != 0:
        print("\n❌ Pandoc nie jest zainstalowany!")
        print("\nZainstaluj pandoc:")
        print("1. Pobierz: https://pandoc.org/installing.html")
        print("2. Lub: choco install pandoc (jeśli masz Chocolatey)")
        print("3. Lub: winget install pandoc")
        return False
    
    # Konwersja do PDF
    cmd = f'pandoc "{input_file}" -o "{output_file}" --pdf-engine=xelatex -V geometry:margin=2cm -V fontsize=11pt --toc --toc-depth=3'
    
    print("\nWykonuję konwersję...")
    result = os.system(cmd)
    
    if result == 0:
        print(f"\n✅ Sukces! Utworzono {output_file}")
        print(f"📍 Lokalizacja: {os.path.abspath(output_file)}")
        return True
    else:
        print(f"\n❌ Błąd podczas konwersji (kod: {result})")
        print("\nMożliwe przyczyny:")
        print("- Brak XeLaTeX (zainstaluj MiKTeX lub TeX Live)")
        print("- Błąd składni w pliku Markdown")
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("KONWERTER MARKDOWN → PDF")
    print("=" * 60)
    convert_to_pdf()
    print("=" * 60)
