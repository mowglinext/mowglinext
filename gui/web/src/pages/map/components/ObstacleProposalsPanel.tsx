import {CheckCircleOutlined, CloseCircleOutlined, WarningOutlined} from "@ant-design/icons";
import {App, Button} from "antd";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import {useApi} from "../../../hooks/useApi.ts";
import {isDigProposal, type ObstacleProposal} from "../utils/obstacleProposals.ts";

interface ObstacleProposalsPanelProps {
    proposals: ObstacleProposal[];
    /// Currently hovered proposal id (shared with the map layer). null = none.
    selectedProposalId: number | null;
    onHoverProposal: (id: number | null) => void;
}

const PROPOSAL_COLOR = '#d48806';

/// Backend failures arrive as a rejected Response carrying the parsed error
/// body, not as an Error instance (see TrackedObstaclesPanel).
const backendMessage = (error: unknown, fallback: string): string =>
    (error as {error?: {error?: string}} | null)?.error?.error
    ?? (error instanceof Error ? error.message : fallback);

/// Lists the PENDING obstacle proposals map_server holds (wheel-slip dig
/// reports). A proposal blocks NOTHING on the robot until it is accepted here:
/// "Accept" calls /map_server_node/promote_obstacle{pending_id} (keepout +
/// coverage hole + saved to the map), "Reject" calls
/// /map_server_node/discard_obstacle. Proposals are session-scoped — they
/// disappear on a ROS2 restart.
export const ObstacleProposalsPanel = ({proposals, selectedProposalId, onHoverProposal}: ObstacleProposalsPanelProps) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    const {modal, notification} = App.useApp();
    const api = useApi();

    if (proposals.length === 0) return null;

    const areaLabel = (proposal: ObstacleProposal) =>
        proposal.areaName || t('mapObstacleProposals.areaFallback', {index: proposal.areaIndex});

    const handleAccept = (proposal: ObstacleProposal) => {
        modal.confirm({
            title: t('mapObstacleProposals.acceptConfirmTitle'),
            content: t('mapObstacleProposals.acceptConfirmBody', {area: areaLabel(proposal)}),
            okText: t('mapObstacleProposals.accept'),
            cancelText: t('mapObstacleProposals.cancel'),
            onOk: async () => {
                try {
                    // pending_id selects map_server's "accept a proposal" path; the
                    // polygon stays empty (the server owns the geometry) but is
                    // sent explicitly so the request never carries a JSON null.
                    const res = await api.mowglinext.callCreate("promote_obstacle", {
                        area_index: proposal.areaIndex,
                        obstacle_id: 0,
                        pending_id: proposal.id,
                        polygon: {points: []},
                        name: proposal.name,
                    });
                    if (res.error) throw new Error(res.error.error);
                    notification.success({message: t('mapObstacleProposals.acceptedSuccess', {area: areaLabel(proposal)})});
                } catch (error: unknown) {
                    notification.error({
                        message: t('mapObstacleProposals.acceptFailed'),
                        description: backendMessage(error, t('mapObstacleProposals.acceptFailed')),
                    });
                    throw error;
                }
            },
        });
    };

    const handleReject = (proposal: ObstacleProposal) => {
        modal.confirm({
            title: t('mapObstacleProposals.rejectConfirmTitle'),
            content: t('mapObstacleProposals.rejectConfirmBody'),
            okText: t('mapObstacleProposals.reject'),
            cancelText: t('mapObstacleProposals.cancel'),
            onOk: async () => {
                try {
                    const res = await api.mowglinext.callCreate("discard_obstacle", {obstacle_id: proposal.id});
                    if (res.error) throw new Error(res.error.error);
                    notification.success({message: t('mapObstacleProposals.rejectedSuccess')});
                } catch (error: unknown) {
                    notification.error({
                        message: t('mapObstacleProposals.rejectFailed'),
                        description: backendMessage(error, t('mapObstacleProposals.rejectFailed')),
                    });
                    throw error;
                }
            },
        });
    };

    return (
        <div style={{display: 'flex', flexDirection: 'column', minWidth: 0}}>
            <div style={{
                padding: '8px 12px',
                fontSize: 12,
                fontWeight: 600,
                color: colors.muted,
                textTransform: 'uppercase',
                letterSpacing: '0.05em',
                borderBottom: `1px solid ${colors.borderSubtle}`,
            }}>
                {t('mapObstacleProposals.header', {count: proposals.length})}
            </div>
            <div style={{padding: '6px 12px', fontSize: 11, color: colors.muted}}>
                {t('mapObstacleProposals.hint')}
            </div>
            <div style={{overflowY: 'auto', flex: 1}}>
                {proposals.map((proposal) => {
                    const isSelected = selectedProposalId === proposal.id;
                    return (
                        <div key={proposal.id}
                            onMouseEnter={() => onHoverProposal(proposal.id)}
                            onMouseLeave={() => onHoverProposal(null)}
                            style={{
                                display: 'flex',
                                flexDirection: 'column',
                                gap: 4,
                                padding: '10px 12px',
                                background: isSelected ? colors.bgElevated : 'transparent',
                                borderLeft: `3px solid ${PROPOSAL_COLOR}`,
                                transition: 'background 0.12s ease',
                            }}>
                            <div style={{display: 'flex', alignItems: 'center', gap: 8}}>
                                <span style={{color: PROPOSAL_COLOR, fontSize: 16, flexShrink: 0}}>
                                    <WarningOutlined aria-hidden="true" />
                                </span>
                                <div style={{fontWeight: 500, fontSize: 13, color: colors.text}}>
                                    {isDigProposal(proposal)
                                        ? t('mapObstacleProposals.digLabel', {id: proposal.id})
                                        : t('mapObstacleProposals.genericLabel', {id: proposal.id})}
                                </div>
                            </div>
                            <div style={{fontSize: 11, color: colors.muted}}>
                                {areaLabel(proposal)}{proposal.name ? ` · ${proposal.name}` : ''}
                            </div>
                            <div style={{display: 'flex', flexWrap: 'wrap', justifyContent: 'flex-end', gap: 4}}>
                                <Button
                                    size="small"
                                    type="text"
                                    icon={<CheckCircleOutlined aria-hidden="true" />}
                                    onClick={() => handleAccept(proposal)}
                                    title={t('mapObstacleProposals.acceptTooltip')}
                                >
                                    {t('mapObstacleProposals.accept')}
                                </Button>
                                <Button
                                    size="small"
                                    type="text"
                                    icon={<CloseCircleOutlined aria-hidden="true" />}
                                    onClick={() => handleReject(proposal)}
                                    title={t('mapObstacleProposals.rejectTooltip')}
                                >
                                    {t('mapObstacleProposals.reject')}
                                </Button>
                            </div>
                        </div>
                    );
                })}
            </div>
        </div>
    );
};
