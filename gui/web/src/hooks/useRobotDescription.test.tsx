import {act,renderHook} from "@testing-library/react";
import {it,expect,vi} from "vitest";
import {ROBOT_URDF} from "../test/robotUrdf";
const state=vi.hoisted(()=>({receive:(data:unknown)=>{void data;}}));
vi.mock("./useWS.ts",()=>({useWS:(_a:unknown,_b:unknown,c:(data:unknown)=>void)=>{
    state.receive=c;return {start:vi.fn(),stop:vi.fn()};
}}));
import {useRobotDescription} from "./useRobotDescription";
it("accepts changed XML, ignores identical republication, and invalidates unsupported descriptions",()=>{
    const {result}=renderHook(()=>useRobotDescription());
    expect(result.current.fromUrdf).not.toBe(true);
    act(()=>state.receive({data:ROBOT_URDF}));
    const first=result.current;
    expect(first.baseLength).toBe(.6);
    act(()=>state.receive({Data:ROBOT_URDF}));
    expect(result.current).toBe(first);
    act(()=>state.receive({data:ROBOT_URDF.replace("0.60 0.45 0.19","0.80 0.45 0.19")}));
    expect(result.current.baseLength).toBe(.8);
    act(()=>state.receive({data:"<robot/>"}));
    expect(result.current.fromUrdf).not.toBe(true);
});
