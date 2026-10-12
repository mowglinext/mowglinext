export class MowingAreaEdit {
    id?: string;
    name: string;
    mowing_order: number;
    orig_mowing_order: number;
    feature_type: string;
    orig_feature_type: string;
    // Only meaningful when converting a workarea/navigation area INTO an obstacle:
    // the polygon was recorded by driving the chassis edge along the object, so it
    // is shrunk once by the chassis half-width (correct_recorded_obstacle).
    shrink_recorded: boolean;
    index: number;

    constructor() {
        this.name = '';
        this.mowing_order = 9999;
        this.orig_mowing_order = 9999;
        this.feature_type = 'workarea';
        this.orig_feature_type = 'workarea';
        this.shrink_recorded = true;
        this.index = -1;
    }
}

export interface AreaListItem {
    id: string;
    name: string;
    ftype: string;
    areaLabel: string;
    mowingOrder?: number;
}
