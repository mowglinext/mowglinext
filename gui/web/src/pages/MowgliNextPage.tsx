import {Collapse, Card, Col, Row} from "antd";
import {useIsMobile} from "../hooks/useIsMobile";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {usePower} from "../hooks/usePower.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useGPS} from "../hooks/useGPS.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {useMowerAction} from "../components/MowerActions.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {computeBatteryPercent} from "../utils/battery.ts";
import {deriveGpsStatus} from "../utils/gpsStatus.ts";
import {
  HeroCard, DashTile, DashCard,
  IconBattery, IconSignal, IconBlades, IconThermo, IconSchedule, IconDiag,
  useTrail, KEYFRAMES_CSS,
} from "../components/dashboard";
import {ImuComponent} from "../components/ImuComponent.tsx";
import {GpsComponent} from "../components/GpsComponent.tsx";
import {WheelTicksComponent} from "../components/WheelTicksComponent.tsx";
import {SystemInfoComponent} from "../components/SystemInfoComponent.tsx";

function useMowerData() {
  const {highLevelStatus} = useHighLevelStatus();
  const power = usePower();
  const status = useStatus();
  const gps = useGPS();
  const emergency = useEmergency();
  const {settings} = useSettings();

  const isCharging = highLevelStatus.is_charging ?? status.is_charging ?? false;
  const isEmergency = highLevelStatus.emergency ?? emergency.active_emergency ?? false;

  const batteryPercent = computeBatteryPercent(
    highLevelStatus.battery_percent, power.v_battery, settings,
  );

  const gpsStatus = deriveGpsStatus(gps.flags);

  const stateName = highLevelStatus.state_name ?? (
    isEmergency ? "EMERGENCY" :
    isCharging ? "CHARGING" :
    status.mower_status != null ? "IDLE" :
    "IDLE"
  );

  const areaPct = (() => {
    if (highLevelStatus.current_path != null && highLevelStatus.current_path > 0 && highLevelStatus.current_path_index != null) {
      return (highLevelStatus.current_path_index / highLevelStatus.current_path) * 100;
    }
    return 0;
  })();

  return {
    state: stateName,
    battery: batteryPercent,
    charging: isCharging,
    emergency: isEmergency,
    gps: gpsStatus.percent,
    gpsLabel: gpsStatus.label,
    vBattery: power.v_battery ?? 0,
    current: power.charge_current ?? 0,
    rpm: status.mower_motor_rpm ?? 0,
    escTemp: status.mower_esc_temperature ?? 0,
    motorTemp: status.mower_motor_temperature ?? 0,
    cpuTemp: 0,
    rain: status.rain_detected ?? false,
    areaPct,
    timeToday: 0,
    dockDistance: 0,
    currentArea: highLevelStatus.current_area != null
      ? `Area ${highLevelStatus.current_area + 1}`
      : undefined,
  };
}

export const MowgliNextPage = () => {
  const isMobile = useIsMobile();
  const {colors} = useThemeMode();
  const mowerAction = useMowerAction();
  const data = useMowerData();

  const batteryTrail = useTrail(data.battery, 32);
  const gpsTrail = useTrail(data.gps, 32);
  const rpmTrail = useTrail(data.rpm, 32);
  const tempTrail = useTrail(data.motorTemp, 32);

  const heroActions = {
    onStart: mowerAction("high_level_control", {Command: 1}),
    onHome: mowerAction("high_level_control", {Command: 2}),
    onPause: mowerAction("mower_logic", {Config: {Bools: [{Name: "manual_pause_mowing", Value: true}]}}),
    onEmergency: mowerAction("emergency", {Emergency: 1}),
    onResumeFromBoundary: mowerAction("high_level_control", {Command: 1}),
    onResetEmergency: mowerAction("emergency", {Emergency: 0}),
  };

  const heroData = {
    state: data.state,
    battery: data.battery,
    charging: data.charging,
    emergency: data.emergency,
    gps: data.gps,
    current: data.current,
    areaPct: data.areaPct,
    timeToday: data.timeToday,
    dockDistance: data.dockDistance,
    rain: data.rain,
    currentArea: data.currentArea,
  };

  const gpsHint = data.gpsLabel;

  if (isMobile) {
    return (
      <div style={{display: 'flex', flexDirection: 'column', gap: 12, paddingBottom: 8}}>
        <style>{KEYFRAMES_CSS}</style>
        <HeroCard data={heroData} compact {...heroActions}/>

        <div style={{display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 10}}>
          <DashTile compact icon={<IconBattery size={12}/>} label="Battery"
                 value={Math.round(data.battery)} unit="%"
                 accent={colors.accent}
                 hint={data.charging ? 'charging' : `${data.vBattery.toFixed(1)} V`}/>
          <DashTile compact icon={<IconSignal size={12}/>} label="GPS"
                 value={Math.round(data.gps)} unit="%"
                 accent={colors.sky} hint={gpsHint}/>
          <DashTile compact icon={<IconBlades size={12}/>} label="Blades"
                 value={data.rpm > 0 ? Math.round(data.rpm) : 'off'}
                 unit={data.rpm > 0 ? 'rpm' : ''}
                 accent={colors.amber}
                 hint={`${data.current.toFixed(1)}A draw`}/>
          <DashTile compact icon={<IconThermo size={12}/>} label="Motor"
                 value={data.motorTemp.toFixed(0)} unit="C"
                 accent={data.motorTemp > 50 ? colors.amber : colors.accent}
                 hint={`ESC ${data.escTemp.toFixed(0)} C`}/>
        </div>

        <Collapse
          defaultActiveKey={[]}
          size="small"
          items={[
            {
              key: "system",
              label: "System Info",
              children: <SystemInfoComponent/>,
            },
            {
              key: "sensors",
              label: "Sensors & Diagnostics",
              children: <Row gutter={[16, 16]}>
                <Col span={24}><Card title="IMU" size="small"><ImuComponent/></Card></Col>
                <Col span={24}><Card title="GPS" size="small"><GpsComponent/></Card></Col>
                <Col span={24}><Card title="Wheel Ticks" size="small"><WheelTicksComponent/></Card></Col>
              </Row>,
            },
          ]}
        />
      </div>
    );
  }

  // Desktop
  return (
    <div style={{display: 'flex', flexDirection: 'column', gap: 16}}>
      <style>{KEYFRAMES_CSS}</style>
      <HeroCard data={heroData} {...heroActions}/>

      {/* At-a-glance tiles */}
      <div style={{display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 12}}>
        <DashTile icon={<IconBattery size={16}/>} label="Battery"
               value={Math.round(data.battery)} unit="%"
               accent={colors.accent} trail={batteryTrail}
               hint={data.charging ? 'charging' : `${data.vBattery.toFixed(1)} V`}/>
        <DashTile icon={<IconSignal size={16}/>} label="GPS"
               value={Math.round(data.gps)} unit="%"
               accent={colors.sky} trail={gpsTrail}
               hint={gpsHint}/>
        <DashTile icon={<IconBlades size={16}/>} label="Blades"
               value={data.rpm > 0 ? Math.round(data.rpm) : 'off'}
               unit={data.rpm > 0 ? 'rpm' : ''}
               accent={colors.amber} trail={rpmTrail}
               hint={`${data.current.toFixed(1)}A draw`}/>
        <DashTile icon={<IconThermo size={16}/>} label="Motor"
               value={data.motorTemp.toFixed(0)} unit="C"
               accent={data.motorTemp > 50 ? colors.amber : colors.accent}
               trail={tempTrail}
               hint={`ESC ${data.escTemp.toFixed(0)} C`}/>
      </div>

      {/* Bottom row: today + next up + health */}
      <div style={{display: 'grid', gridTemplateColumns: '1.2fr 1fr 1fr', gap: 14}}>
        {/* Today's work */}
        <DashCard>
          <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 14}}>
            <div style={{fontSize: 14, fontWeight: 600}}>Today's work</div>
            <div style={{fontSize: 11, color: colors.textMuted}}>
              {new Date().toLocaleDateString(undefined, {weekday: 'long'})}
            </div>
          </div>
          <div style={{display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 14, marginBottom: 16}}>
            <div>
              <div style={{fontSize: 24, fontWeight: 700, letterSpacing: '-0.02em'}}>
                {data.areaPct.toFixed(0)}<span style={{fontSize: 13, color: colors.textDim, marginLeft: 3}}>%</span>
              </div>
              <div style={{fontSize: 11, color: colors.textDim, marginTop: 2}}>zone progress</div>
            </div>
            <div>
              <div style={{fontSize: 24, fontWeight: 700, letterSpacing: '-0.02em'}}>
                {data.currentArea ? '1' : '0'}<span style={{fontSize: 13, color: colors.textDim, marginLeft: 3}}>zones</span>
              </div>
              <div style={{fontSize: 11, color: colors.textDim, marginTop: 2}}>active today</div>
            </div>
          </div>
        </DashCard>

        {/* Next up */}
        <DashCard>
          <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 14}}>
            <div style={{fontSize: 14, fontWeight: 600}}>Next up</div>
            <IconSchedule size={16}/>
          </div>
          <div style={{fontSize: 13, color: colors.textDim, lineHeight: 1.6}}>
            Check the Schedule page to set up automated mowing runs.
          </div>
        </DashCard>

        {/* Health check */}
        <DashCard>
          <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 14}}>
            <div style={{fontSize: 14, fontWeight: 600}}>Health check</div>
            <IconDiag size={16}/>
          </div>
          <div style={{display: 'flex', flexDirection: 'column', gap: 8}}>
            {[
              {k: 'GPS signal', ok: data.gps > 0, note: gpsHint},
              {k: data.rain ? 'Raining' : 'No rain', ok: !data.rain, note: data.rain ? 'mowing paused' : 'dry out there'},
              {k: data.emergency ? 'Emergency active' : 'No emergency', ok: !data.emergency, note: data.emergency ? 'needs reset' : 'all clear'},
              {k: `Motor ${data.motorTemp.toFixed(0)} C`, ok: data.motorTemp < 55, note: data.motorTemp >= 55 ? 'running hot' : 'normal'},
            ].map((r, i) => (
              <div key={i} style={{display: 'flex', alignItems: 'center', gap: 10}}>
                <div style={{
                  width: 8, height: 8, borderRadius: 4,
                  background: r.ok ? colors.accent : colors.danger,
                  flexShrink: 0,
                }}/>
                <div style={{flex: 1, minWidth: 0}}>
                  <div style={{fontSize: 13, fontWeight: 600}}>{r.k}</div>
                  <div style={{fontSize: 11, color: colors.textMuted}}>{r.note}</div>
                </div>
              </div>
            ))}
          </div>
        </DashCard>
      </div>

      {/* Sensors accordion */}
      <Collapse
        defaultActiveKey={[]}
        items={[
          {
            key: "system",
            label: "System Info",
            children: <SystemInfoComponent/>,
          },
          {
            key: "sensors",
            label: "Sensors & Diagnostics",
            children: <Row gutter={[16, 16]}>
              <Col span={24}><Card title="IMU" size="small"><ImuComponent/></Card></Col>
              <Col lg={12} xs={24}><Card title="GPS" size="small"><GpsComponent/></Card></Col>
              <Col lg={12} xs={24}><Card title="Wheel Ticks" size="small"><WheelTicksComponent/></Card></Col>
            </Row>,
          },
        ]}
      />
    </div>
  );
};

export default MowgliNextPage;
