# HTML Design Docs

Open `docs/design/html/index.html` in a browser.

Pages are static HTML and use Mermaid via CDN to render diagrams.
If your browser blocks local module imports from `file://`, serve the repo root with a static file server, for example:

```bash
python3 -m http.server 8080
```

Then open:

- `http://localhost:8080/docs/design/html/index.html`
