export interface PdfOptions {
    filename?: string;
}

const getHtml2PdfOptions = (filename?: string) => ({
    margin: 0.5,
    filename: filename ?? 'document.pdf',
    image: { type: 'jpeg' as const, quality: 0.98 },
    html2canvas: { scale: 2, useCORS: true, backgroundColor: "#ffffff", windowWidth: 800 },
    jsPDF: { unit: 'in', format: 'letter' as const, orientation: 'portrait' as const }
});

const createPrintWrapper = (htmlContent: string) => {
    const printWrapper = document.createElement("div");
    printWrapper.style.position = "absolute";
    printWrapper.style.left = "-9999px";
    printWrapper.style.top = "0px";
    printWrapper.className = "light"; // Prevent dark mode variables from bleeding into the physical PDF

    const element = document.createElement("div");
    element.innerHTML = htmlContent;
    // Assign standard prose layout and explicitly white/black print colors
    element.className = "prose prose-sm max-w-none p-12 bg-white text-black";
    element.style.width = "800px";
    element.style.backgroundColor = "white";
    element.style.color = "black";

    printWrapper.appendChild(element);
    document.body.appendChild(printWrapper);

    return { printWrapper, element };
};

export async function downloadPdfFromHtml(htmlContent: string, options: PdfOptions = {}): Promise<void> {
    // @ts-ignore - Dynamic import to prevent SSR issues with window/document
    const html2pdf = (await import('html2pdf.js')).default;

    const { printWrapper, element } = createPrintWrapper(htmlContent);

    try {
        const opt = getHtml2PdfOptions(options.filename);
        await html2pdf().set(opt).from(element).save();
    } finally {
        document.body.removeChild(printWrapper);
    }
}

export async function generatePdfBlobFromHtml(htmlContent: string, options: PdfOptions = {}): Promise<Blob> {
    // @ts-ignore - Dynamic import to prevent SSR issues with window/document
    const html2pdf = (await import('html2pdf.js')).default;

    const { printWrapper, element } = createPrintWrapper(htmlContent);

    try {
        const opt = getHtml2PdfOptions(options.filename);
        const pdfBlob = await html2pdf().set(opt).from(element).output('blob');
        return pdfBlob;
    } finally {
        document.body.removeChild(printWrapper);
    }
}
