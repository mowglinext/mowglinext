import React, {useRef, useState} from 'react';
import {Alert, Button, Card, Input, Space, Table, Typography, Popconfirm, Select, Switch} from 'antd';
import {CodeOutlined, DeleteOutlined, PlusOutlined} from '@ant-design/icons';
import {useTranslation} from 'react-i18next';
import type {SettingsValues} from '../../types/settings.generated';
import {parseAdvancedNumber, structuredAdvancedPreview, type AdvancedScalar, type AdvancedKeyError} from './advancedSettings';

const {Text} = Typography;
type AdvancedRow = {key: string; name: string; value: unknown};
type Props = {
    values: SettingsValues;
    advancedKeys: string[];
    onChange: (key: string, value: AdvancedScalar | null) => void;
    validateNewKey: (key: string) => AdvancedKeyError | undefined;
};
function NumberEditor({name, value, onChange}: {name: string; value: number; onChange: Props['onChange']}) {
    const {t} = useTranslation();
    const [text, setText] = useState(String(value));
    const [invalid, setInvalid] = useState(false);
    const committed = useRef(value);
    const commit = () => {
        const parsed = parseAdvancedNumber(text);
        if (parsed === undefined) {
            setInvalid(true);
            setText(String(value)); // Invalid/partial drafts never change the parameter's type.
        } else {
            setInvalid(false);
            if (!Object.is(parsed, committed.current)) {
                committed.current = parsed; // Enter followed by blur is one logical edit.
                onChange(name, parsed);
            }
        }
    };
    return <div>
        <Input size="small" aria-label={t('settingsAdvanced.valueFor', {key: name})}
            value={text} status={invalid ? 'error' : undefined}
            onChange={event => {setText(event.target.value); setInvalid(false);}}
            onBlur={commit} onPressEnter={commit}/>
        <Text type={invalid ? 'danger' : 'secondary'} style={{fontSize: 11}}>
            {t(invalid ? 'settingsAdvanced.invalidNumber' : 'settingsAdvanced.numberCommitHint')}
        </Text>
    </div>;
}
export const AdvancedSection: React.FC<Props> = ({values, advancedKeys, onChange, validateNewKey}) => {
    const {t} = useTranslation();
    const [newKey, setNewKey] = useState('');
    const [newValue, setNewValue] = useState('');
    const [newType, setNewType] = useState<'string' | 'number' | 'boolean'>('string');
    const [newBoolean, setNewBoolean] = useState(false);
    const keyError = validateNewKey(newKey);
    const numericValue = parseAdvancedNumber(newValue);
    const validValue = newType !== 'number' || numericValue !== undefined;
    const handleAdd = () => {
        if (validateNewKey(newKey) !== undefined || !validValue) return;
        const value = newType === 'boolean' ? newBoolean : newType === 'number' ? numericValue : newValue;
        if (value === undefined) return;
        onChange(newKey, value);
        setNewKey(''); setNewValue(''); setNewBoolean(false);
    };
    const dataSource: AdvancedRow[] = advancedKeys.filter(key => values[key] != null).sort()
        .map(key => ({key, name: key, value: values[key]}));
    const renderValue = (row: AdvancedRow) => {
        const value = row.value;
        if (typeof value === 'string') return <Input size="small" aria-label={t('settingsAdvanced.valueFor', {key: row.name})}
            value={value} onChange={event => onChange(row.name, event.target.value)}/>;
        if (typeof value === 'boolean') return <Switch size="small" aria-label={t('settingsAdvanced.valueFor', {key: row.name})}
            checked={value} onChange={checked => onChange(row.name, checked)}/>;
        if (typeof value === 'number' && Number.isFinite(value)) return <NumberEditor
            key={`${row.name}:${Object.is(value, -0) ? '-0' : value}`} name={row.name} value={value} onChange={onChange}/>;
        const preview = structuredAdvancedPreview(value);
        return <div><Text type="secondary">{t(preview === undefined ? 'settingsAdvanced.unsupportedReadOnly' : 'settingsAdvanced.structuredReadOnly')}</Text>
            {preview !== undefined && <pre style={{fontSize: 11, whiteSpace: 'pre-wrap', maxHeight: 120, overflow: 'auto'}}>{preview}</pre>}
        </div>;
    };
    return <div>
        <Alert type="info" showIcon icon={<CodeOutlined/>} message={t('settingsAdvanced.advancedParameters')}
            description={t('settingsAdvanced.advancedParametersDescription')} style={{marginBottom: 16}}/>
        <Card size="small" style={{marginBottom: 16}}>
            <Table<AdvancedRow> dataSource={dataSource} pagination={false} size="small"
                locale={{emptyText: t('settingsAdvanced.noExtraParameters')}} columns={[
                    {title: t('settingsAdvanced.parameter'), dataIndex: 'name', key: 'name', width: '40%',
                        render: (_: unknown, row: AdvancedRow) => <Text code style={{fontSize: 11}}>{row.name}</Text>},
                    {title: t('settingsAdvanced.value'), dataIndex: 'value', key: 'value',
                        render: (_: unknown, row: AdvancedRow) => renderValue(row)},
                    {title: '', key: 'actions', width: 40, render: (_: unknown, row: AdvancedRow) => <Popconfirm
                        title={t('settingsAdvanced.removeThisParameter')} onConfirm={() => onChange(row.name, null)}
                        okText={t('settingsAdvanced.remove')} cancelText={t('settingsAdvanced.cancel')}>
                        <Button type="text" size="small" danger aria-label={t('settingsAdvanced.deleteKey', {key: row.name})} icon={<DeleteOutlined/>}/>
                    </Popconfirm>},
                ]}/>
        </Card>
        <Card size="small" title={t('settingsAdvanced.addParameter')} style={{marginBottom: 16}}>
            <Space wrap>
                <Input placeholder="parameter_name" aria-label={t('settingsAdvanced.newKey')} value={newKey}
                    onChange={event => setNewKey(event.target.value)} onPressEnter={handleAdd}/>
                <Select aria-label={t('settingsAdvanced.valueType')} value={newType} onChange={setNewType}
                    options={(['string', 'number', 'boolean'] as const).map(type => ({value: type, label: t(`settingsAdvanced.types.${type}`)}))}/>
                {newType === 'boolean' ? <Switch aria-label={t('settingsAdvanced.newValue')} checked={newBoolean} onChange={setNewBoolean}/>
                    : <Input placeholder={t('settingsAdvanced.valuePlaceholder')} aria-label={t('settingsAdvanced.newValue')}
                        value={newValue} onChange={event => setNewValue(event.target.value)} onPressEnter={handleAdd}/>}
                <Button type="primary" icon={<PlusOutlined/>} onClick={handleAdd} disabled={keyError !== undefined || !validValue}>
                    {t('settingsAdvanced.add')}
                </Button>
            </Space>
            {newKey !== '' && keyError !== undefined && <Alert type="error" message={t(`settingsAdvanced.${keyError}`)}/>}
            {newType === 'number' && newValue !== '' && !validValue && <Alert type="error" message={t('settingsAdvanced.invalidNumber')}/>}
        </Card>
    </div>;
};
