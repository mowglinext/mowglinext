import {useEffect, useState} from 'react';
import {Popover} from 'antd';
import {useTranslation} from 'react-i18next';
import {useTopic} from '../hooks/useTopic';
import type {Status} from '../types/ros';

/** Command intent, independently aged using receipt time (also works in simulation). */
export function BladeDirectionDisplay({compact = false}: {compact?: boolean}) {
    const {t} = useTranslation();
    const {data, lastMessageAt} = useTopic<Status>('status', {}, {withTimestamp: true});
    const [now, setNow] = useState(() => Date.now());
    useEffect(() => {
        const timer = setInterval(() => setNow(Date.now()), 1000);
        return () => clearInterval(timer);
    }, []);
    const fresh = lastMessageAt !== null && now - lastMessageAt < 5000;
    const value = fresh ? data.blade_requested_direction : undefined;
    const key = value === 'forward' || value === 'reverse' || value === 'off' ? value : 'unknown';
    const label = t(`bladeDirection.${key}`);
    const description = t('bladeDirection.summary', {direction: label});
    return compact ? (
        <Popover content={<span style={{display: 'block', maxWidth: 280}}>{t('bladeDirection.help')}</span>}
                 trigger="click">
            <button type="button" aria-label={description}
                    style={{color: 'inherit', background: 'none', border: 0, padding: 0, font: 'inherit', cursor: 'help'}}>
                {description}
            </button>
        </Popover>
    ) : (
        <div title={t('bladeDirection.help')} aria-label={description}>
            <div style={{fontSize: 14, opacity: 0.65, marginBottom: 4}}>{t('bladeDirection.title')}</div>
            <div style={{fontSize: 24}}>{label}</div>
            <div style={{fontSize: 12, opacity: 0.65}}>{t('bladeDirection.help')}</div>
        </div>
    );
}
