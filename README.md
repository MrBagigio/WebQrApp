# WebQrApp

Una semplice applicazione web statica per il tracciamento AR con marker ArUco.

## Pubblicazione su GitHub Pages

Questo progetto può essere pubblicato su GitHub Pages per ottenere un server statico
sempre attivo. È già presente un workflow in `.github/workflows/pages.yml` che
invia automaticamente i file del repository al branch `gh-pages` ogni volta che
vengono fatti push sul branch `main`.

1. Crea un repository su GitHub e spingi (push) il progetto su quel repository
   (usa il branch `main`).
2. Vai nelle impostazioni del repository, sezione **Pages**, e scegli il branch
   `gh-pages` come sorgente di pubblicazione. Il workflow si occuperà del resto.
3. Il sito sarà accessibile all'indirizzo
   `https://<tuo-nome-utente>.github.io/<nome-del-repo>/`.

> Nota: se preferisci servire solo una cartella `docs/`, modifica il valore
> `folder:` nel workflow e posiziona lì i file. In questo progetto la cartella root
> viene distribuita.
