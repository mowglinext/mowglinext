import {Flex, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {DockGraphic} from "./DockGraphic";
import {dockBounds} from "./dockGeometry";

export function DockPreview() {
    const {t}=useTranslation();
    return <section data-testid="dock-preview" style={{marginTop:16,borderTop:"1px solid rgba(128,174,157,.2)",paddingTop:12}}>
        <Typography.Text strong>{t("mowerVisual.dockStyle")}</Typography.Text>
        <Flex wrap gap={16}>
            {(["top","side"] as const).map(view=>{
                const b=dockBounds(view);
                return <figure key={view} style={{margin:0,flex:"1 1 160px",minWidth:0}}>
                    <svg role="img" aria-label={t("mowerVisual.dockStyle")+" · "+t("mowerVisual."+view)}
                        viewBox={[b.x+b.width/2-.365,b.y+b.height/2-.365,.73,.73].join(" ")}
                        width="100%" height={170}><DockGraphic view={view}/></svg>
                </figure>;
            })}
        </Flex>
        <Typography.Text type="secondary" style={{fontSize:12}}>{t("mowerVisual.dockNote")}</Typography.Text>
    </section>;
}
