import {Form, Input, InputNumber, Modal, Select} from "antd";
import {useTranslation} from "react-i18next";
import {MowingAreaEdit} from "../utils/types.ts";

interface EditAreaModalProps {
    open: boolean;
    area: MowingAreaEdit;
    onChange: (area: MowingAreaEdit) => void;
    onSave: () => void;
    onCancel: () => void;
}

const AREA_TYPE_OPTIONS = [
    {value: 'workarea', labelKey: 'mapEditArea.areaTypeMowing'},
    {value: 'navigation', labelKey: 'mapEditArea.areaTypeNavigation'},
    {value: 'obstacle', labelKey: 'mapEditArea.areaTypeObstacle'},
];

export const EditAreaModal = ({open, area, onChange, onSave, onCancel}: EditAreaModalProps) => {
    const {t} = useTranslation();
    return (
        <Modal
            open={open}
            title={area.name ? t('mapEditArea.titleEditNamed', {name: area.name}) : t('mapEditArea.titleEdit')}
            okText={t('mapEditArea.save')}
            cancelText={t('mapEditArea.cancel')}
            onOk={onSave}
            onCancel={onCancel}
            destroyOnClose
        >
            <Form layout="vertical" style={{marginTop: 16}}>
                <Form.Item label={t('mapEditArea.areaType')}>
                    <Select
                        value={area.feature_type}
                        onChange={(v) => onChange({...area, feature_type: v})}
                        options={AREA_TYPE_OPTIONS.map((o) => ({value: o.value, label: t(o.labelKey)}))}
                    />
                </Form.Item>
                {area.feature_type === 'workarea' && (
                    <Form.Item label={t('mapEditArea.areaName')}>
                        <Input
                            key="areaname"
                            placeholder={t('mapEditArea.areaNamePlaceholder')}
                            value={area.name}
                            onChange={(e) => onChange({...area, name: e.target.value})}
                            autoFocus
                        />
                    </Form.Item>
                )}
                {area.feature_type === 'workarea' && (
                    <Form.Item label={t('mapEditArea.mowingOrder')}>
                        <InputNumber
                            key="mowingorder"
                            min={1}
                            value={area.mowing_order}
                            onChange={(v) => onChange({...area, mowing_order: v ?? 9999})}
                            style={{width: '100%'}}
                        />
                    </Form.Item>
                )}
            </Form>
        </Modal>
    );
};
