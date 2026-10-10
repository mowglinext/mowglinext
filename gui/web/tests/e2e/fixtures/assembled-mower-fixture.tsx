import {useState} from "react";
import {createRoot} from "react-dom/client";
import Map,{useMap} from "react-map-gl/mapbox";
import mapboxgl from "mapbox-gl";
import "mapbox-gl/dist/mapbox-gl.css";
import "../../../src/i18n";
import {StyledDockMarker} from "../../../src/pages/map/components/StyledDockMarker";
import {AssembledMowerMarker} from "../../../src/pages/map/components/AssembledMowerMarker";
import {parseRobotUrdf} from "../../../src/utils/robotModel";
import {ROBOT_URDF} from "../../../src/test/robotUrdf";
const CENTER:[number,number]=[-1.2,52];
mapboxgl.accessToken="pk.test-no-network-token";
declare global {interface Window {assembledMowerTest?:{map:mapboxgl.Map;setHeading:(n:number)=>void}}}
function Fixture(){
    const {current:map}=useMap();
    const [heading,setHeading]=useState(Math.PI/2);
    if(map) window.assembledMowerTest={map:map.getMap(),setHeading};
    return <><StyledDockMarker longitude={CENTER[0]} latitude={CENTER[1]} headingRad={heading}/><AssembledMowerMarker robot={parseRobotUrdf(ROBOT_URDF)!} longitude={CENTER[0]} latitude={CENTER[1]} headingRad={heading}/></>;
}
createRoot(document.getElementById("root")!).render(<Map initialViewState={{longitude:CENTER[0],latitude:CENTER[1],zoom:24}}
    maxZoom={24} style={{width:1000,height:700}} mapStyle={{version:8,sources:{},layers:[{id:"background",type:"background",paint:{"background-color":"#17352b"}}]}}
    attributionControl={false}><Fixture/></Map>);
