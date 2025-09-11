# VentCon2 LaTeX Documentation

This folder contains comprehensive LaTeX documentation for the VentCon2 pressure control system.

## Documents

### 1. VentCon2_Documentation.tex
The main technical documentation covering:
- **System Architecture**: Complete overview of the embedded system design
- **Class Documentation**: Detailed description of each C++ class
- **Dependencies**: Visual dependency diagrams and relationships
- **Control Theory**: PID control implementation and auto-tuning algorithms
- **Real-time Operation**: FreeRTOS task management and timing analysis
- **Web Interface**: HTTP API and user interface documentation
- **Configuration**: System parameters and persistent storage
- **Performance**: Metrics, timing requirements, and safety mechanisms

### 2. Dependencies_Matrix.tex
A technical supplement providing:
- **Dependency Matrix**: Visual table showing class relationships
- **Initialization Order**: Required startup sequence
- **Coupling Analysis**: Architecture quality assessment
- **Memory Impact**: External library dependencies

## Requirements

### LaTeX Distribution
Install a complete LaTeX distribution such as:
- **Windows**: MiKTeX or TeX Live
- **macOS**: MacTeX
- **Linux**: TeX Live (available in most package managers)

### Required Packages
The following LaTeX packages are required:
```latex
\usepackage{geometry}      % Page layout
\usepackage{graphicx}      % Graphics inclusion
\usepackage{tikz}          % Diagrams and drawings
\usepackage{pgfplots}      % Plots and charts
\usepackage{booktabs}      % Professional tables
\usepackage{listings}      % Code syntax highlighting
\usepackage{xcolor}        % Color support
\usepackage{hyperref}      % Hyperlinks and references
\usepackage{fancyhdr}      % Headers and footers
```

## Compilation

### Method 1: Using Make (Recommended)
If you have `make` installed:

```bash
# Build both documents
make all

# Build main documentation only
make main

# Build dependencies matrix only  
make deps

# View main documentation (Windows)
make view

# Clean auxiliary files
make clean

# Clean everything including PDFs
make cleanall

# Show help
make help
```

### Method 2: Manual Compilation

For the main documentation:
```bash
pdflatex VentCon2_Documentation.tex
pdflatex VentCon2_Documentation.tex  # Second run for TOC
```

For the dependencies matrix:
```bash
pdflatex Dependencies_Matrix.tex
```

### Method 3: Using VS Code
If you have the LaTeX Workshop extension:
1. Open the `.tex` file in VS Code
2. Use `Ctrl+Alt+B` to build
3. Use `Ctrl+Alt+V` to view the PDF

## Output Files

After compilation, you'll have:
- `VentCon2_Documentation.pdf` - Main technical documentation (~25 pages)
- `Dependencies_Matrix.pdf` - Class dependency analysis (~3 pages)

## Editing the Documentation

### Adding New Sections
To add new sections to the main document:
```latex
\section{New Section Title}
\subsection{Subsection}
Content here...
```

### Adding Code Examples
Use the `listings` environment for code:
```latex
\begin{lstlisting}[caption=Code Description]
// Your C++ code here
void function() {
    // Implementation
}
\end{lstlisting}
```

### Adding Diagrams
Use TikZ for technical diagrams:
```latex
\begin{figure}[H]
\centering
\begin{tikzpicture}
    % Your diagram code
\end{tikzpicture}
\caption{Diagram Description}
\end{figure}
```

### Adding Tables
Use `booktabs` for professional tables:
```latex
\begin{table}[H]
\centering
\begin{tabular}{@{}ll@{}}
\toprule
\textbf{Column 1} & \textbf{Column 2} \\
\midrule
Data 1 & Data 2 \\
\bottomrule
\end{tabular}
\caption{Table Description}
\end{table}
```

## Troubleshooting

### Common Issues

1. **Missing Packages**
   - Install missing packages through your LaTeX distribution
   - MiKTeX will usually prompt to install automatically

2. **Compilation Errors**
   - Check the `.log` file for detailed error messages
   - Ensure all required packages are installed
   - Run `pdflatex` twice for cross-references

3. **TikZ/PGFPlots Issues**
   - Ensure you have the latest version of these packages
   - Check for syntax errors in diagram code

4. **Long Compilation Times**
   - TikZ diagrams can be slow to compile
   - Consider using draft mode during editing: `\documentclass[draft]{article}`

### Getting Help
- LaTeX documentation: https://www.latex-project.org/help/documentation/
- TikZ manual: Available in most LaTeX distributions
- TeX Stack Exchange: https://tex.stackexchange.com/

## Maintenance

### Updating Documentation
When modifying the VentCon2 codebase:
1. Update class descriptions in the main document
2. Modify the dependency matrix if class relationships change
3. Update version numbers and timestamps
4. Recompile both documents

### Version Control
- Include the `.tex` source files in version control
- Consider including the generated PDFs for easy access
- Use `.gitignore` for auxiliary files (`*.aux`, `*.log`, etc.)

## Integration with Development

### Automated Documentation
Consider setting up CI/CD to automatically generate documentation:
1. Trigger on code changes
2. Extract class information automatically
3. Generate updated LaTeX documents
4. Deploy PDFs to documentation site

### Documentation Standards
- Keep documentation synchronized with code changes
- Include version numbers and build dates
- Use consistent formatting and terminology
- Regular review and updates for accuracy

---

**Note**: This documentation system provides a professional, publication-quality technical reference for the VentCon2 system. The LaTeX format ensures consistent formatting, professional appearance, and easy maintenance of complex technical content.
