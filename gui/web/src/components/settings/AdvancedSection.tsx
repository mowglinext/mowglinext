import React, { useState } from "react";
import { Alert, Button, Card, Input, Space, Table, Typography, Popconfirm } from "antd";
import { CodeOutlined, DeleteOutlined, PlusOutlined } from "@ant-design/icons";

const { Text } = Typography;

type Props = {
    values: Record<string, any>;
    advancedKeys: string[];
    onChange: (key: string, value: any) => void;
};

export const AdvancedSection: React.FC<Props> = ({ values, advancedKeys, onChange }) => {
    const [newKey, setNewKey] = useState("");
    const [newValue, setNewValue] = useState("");

    const handleAdd = () => {
        const key = newKey.trim();
        if (!key) return;
        // Try to parse as number or boolean
        let parsed: any = newValue;
        if (newValue === "true") parsed = true;
        else if (newValue === "false") parsed = false;
        else if (!isNaN(Number(newValue)) && newValue.trim() !== "") parsed = Number(newValue);
        onChange(key, parsed);
        setNewKey("");
        setNewValue("");
    };

    const handleDelete = (key: string) => {
        // Send an explicit null so the key survives JSON.stringify and reaches
        // the backend, which deletes null-valued keys from the YAML. Using
        // undefined here made JSON.stringify drop the key entirely, so the
        // backend's merge preserved the on-disk value and the delete was a
        // silent no-op.
        onChange(key, null);
    };

    const handleValueChange = (key: string, val: string) => {
        let parsed: any = val;
        if (val === "true") parsed = true;
        else if (val === "false") parsed = false;
        else if (!isNaN(Number(val)) && val.trim() !== "") parsed = Number(val);
        onChange(key, parsed);
    };

    const dataSource = advancedKeys
        .filter((k) => values[k] !== undefined)
        .sort()
        .map((key) => ({
            key,
            name: key,
            value: values[key],
        }));

    return (
        <div>
            <Alert
                type="info"
                showIcon
                icon={<CodeOutlined />}
                message="Advanced parameters"
                description="These are raw YAML parameters not covered by the sections above. Edit with care — invalid values can break the robot configuration."
                style={{ marginBottom: 16 }}
            />

            <Card size="small" style={{ marginBottom: 16 }}>
                <Table
                    dataSource={dataSource}
                    pagination={false}
                    size="small"
                    locale={{ emptyText: "No extra parameters defined" }}
                    columns={[
                        {
                            title: "Parameter",
                            dataIndex: "name",
                            key: "name",
                            width: "40%",
                            render: (name: string) => (
                                <Text code style={{ fontSize: 11 }}>{name}</Text>
                            ),
                        },
                        {
                            title: "Value",
                            dataIndex: "value",
                            key: "value",
                            render: (val: any, record: any) => (
                                <Input
                                    size="small"
                                    value={String(val ?? "")}
                                    onChange={(e) => handleValueChange(record.name, e.target.value)}
                                    style={{ fontFamily: "monospace", fontSize: 12 }}
                                />
                            ),
                        },
                        {
                            title: "",
                            key: "actions",
                            width: 40,
                            render: (_: any, record: any) => (
                                <Popconfirm
                                    title="Remove this parameter?"
                                    onConfirm={() => handleDelete(record.name)}
                                    okText="Remove"
                                    cancelText="Cancel"
                                >
                                    <Button
                                        type="text"
                                        size="small"
                                        danger
                                        icon={<DeleteOutlined />}
                                    />
                                </Popconfirm>
                            ),
                        },
                    ]}
                />
            </Card>

            <Card size="small" title="Add Parameter" style={{ marginBottom: 16 }}>
                <Space.Compact style={{ width: "100%" }}>
                    <Input
                        placeholder="parameter_name"
                        value={newKey}
                        onChange={(e) => setNewKey(e.target.value.toLowerCase().replace(/[^a-z0-9_]/g, "_"))}
                        style={{ width: "40%", fontFamily: "monospace", fontSize: 12 }}
                        onPressEnter={handleAdd}
                    />
                    <Input
                        placeholder="value"
                        value={newValue}
                        onChange={(e) => setNewValue(e.target.value)}
                        style={{ flex: 1, fontFamily: "monospace", fontSize: 12 }}
                        onPressEnter={handleAdd}
                    />
                    <Button
                        type="primary"
                        icon={<PlusOutlined />}
                        onClick={handleAdd}
                        disabled={!newKey.trim()}
                    >
                        Add
                    </Button>
                </Space.Compact>
            </Card>
        </div>
    );
};
