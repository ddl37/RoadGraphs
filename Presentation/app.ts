// import * as THREE from "three"
// import { OrbitControls } from "./OrbitControls"
// import { VertexNormalsHelper } from "./VertexNormalsHelpers"
// import { CSS2DRenderer, CSS2DObject } from "./CSS2DRenderer"

import * as d3 from "d3"

import Reveal from "reveal.js"
import Markdown from 'reveal.js/plugin/markdown/markdown.esm.js';
import RevealHighlight from "reveal.js/plugin/highlight/highlight.js"
import RevealNotes from "reveal.js/plugin/notes/notes"
import RevealMath3 from "./mathjax3-plugin"

import "reveal.js/dist/reveal.css"
import "reveal.js/dist/theme/black.css"
import "reveal.js/plugin/highlight/monokai.css"


let deck = new Reveal(document.querySelector(".reveal"), {
    embedded: true,
    plugins: [Markdown, RevealMath3, RevealHighlight, RevealNotes, RevealFullscreen]
})


fetch("copy.md").then(response => {
    response.text().then(text => {
        let slide_content = d3.create("script")
            .attr("type", "text/template")
            .text(text)

        d3.select("#foo_content").node().appendChild(slide_content.node())

        deck.initialize()
    })
})

/// RENDER THE 3D INTERACTIVE MAP
// import harp from "@here/harp.gl"
// import { MapView } from "@here/harp-mapview"
// import { GeoCoordinates } from "@here/harp-geoutils"
// import { MapControls, MapControlsUI } from "@here/harp-map-controls"
// import { VectorTileDataSource } from "@here/harp-vectortile-datasource"

const canvas = document.getElementById('map');
const mapView = new harp.MapView({
    canvas,
    // theme: "https://unpkg.com/@here/harp-map-theme@latest/resources/berlin_tilezen_effects_streets.json",
    theme: "https://unpkg.com/@here/harp-map-theme@latest/resources/berlin_tilezen_effects_outlines.json",
});

window.toNZ = () => {
    mapView.lookAt({
        target: new harp.GeoCoordinates(-36.9323, 174.8616),
        zoomLevel: 14,
        tilt: 40,
    });
}

setTimeout(() => {
    window.toNZ()
}, 5000)

const mapControls = new harp.MapControls(mapView);
const ui = new harp.MapControlsUI(mapControls);
canvas.parentElement.appendChild(ui.domElement);

mapView.resize(window.innerWidth, window.innerHeight);
window.onresize = () => mapView.resize(window.innerWidth, window.innerHeight);

// obfuscate the API key to prevent bots scraping it off Github
// I'm probably not supposed to publish it?
let krv = "UiPOkLeGJlBpyBGtum" + "CcChs_v0LOog9tZZMyrr4fVtp"
let k = krv.split("").reverse().join("")

const vectorTileDataSource = new harp.VectorTileDataSource({
    authenticationCode: k,
});
mapView.addDataSource(vectorTileDataSource);

// TODO: change stylesheet to show roads better, more contrast

async function load_sensor_flow_data() {
    const res = await fetch("Maps/sample.geojson");
    const data = await res.json();
    const dataProvider = new harp.GeoJsonDataProvider("sensor-flows", data);
    const geoJsonDataSource = new harp.VectorTileDataSource({
        dataProvider,
        name: "sensor-flows",
        styleSetName: "geojson",
    });

    await mapView.addDataSource(geoJsonDataSource);
    const theme: harp.Theme = {
        styles: {
            geojson: [
                {
                    when: ["==", ["geometry-type"], "Point"],
                    technique: "circles",
                    renderOrder: 10000,
                    color: "#FF0000",
                    size: 15,
                },
                {
                    when: ["all",
                        ["==", ["geometry-type"], "LineString"],
                        // ["<=", ]
                    ],
                    renderOrder: 1000,
                    technique: "solid-line",
                    // color: "#D73060",
                    color: ["get", "color"],
                    opacity: 1,
                    metricUnit: "Pixel",
                    lineWidth: 2,
                },
            ],
        },
    };

    geoJsonDataSource.setTheme(theme);
    // mapView.lookAt({
    //     target: new harp.GeoCoordinates(1.278676, 103.850216),
    //     tilt: 45,
    //     zoomLevel: 16,
    // });
    mapView.update();
}

load_sensor_flow_data().then(console.log).catch(console.error)
