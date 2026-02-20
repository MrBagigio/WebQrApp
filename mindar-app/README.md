# MindAR QR Demo

Questa è una versione minimale pensata per funzionare con il codice QR generato da
`mind-ar` (`assets/targets.mind`).

### Come usare
1. Metti il tuo file GLB/glTF della casetta in `mindar-app/assets/house.glb`.
2. Carica la cartella `mindar-app` su un server HTTPS (GitHub Pages, `python -m http.server` + ngrok, ecc.).
3. Visita `.../mindar-app/index.html` con la fotocamera attiva.
4. Punta la fotocamera sul QR/immagine target stampata; il modello verrà ancorato a.

La versione **non usa** i marker ArUco e non dipende da `restoration-engine.js`.
È completamente separata dal resto del progetto.
