import {Spin} from "antd";

export function Spinner() {
    return (
        <div style={{
            position: "fixed",
            inset: 0,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            background: "var(--bg-deep)",
            zIndex: 10,
        }}>
            <Spin size={"large"}/>
        </div>
    );
}
