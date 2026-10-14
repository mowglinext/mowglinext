/** Single source for mower artwork and registration. Add artwork and metadata here;
 * selectors, schema enum, IDs, map derivatives and rendering derive from it.
 * Bounds exclude padding; frames also include hidden physical chassis extents.
 * Labels live with each entry so new styles need no separate locale lists.
 */
export const APPEARANCE_REGISTRY = {
    "defaultShell": "sculpted",
    "defaultDock": "standard",
    "shells": {
        "tron": {
            "size": [2059, 764],
            "top": [136, 32, 553, 702],
            "side": [865, 204, 1111, 440],
            "files": {"top": "tron.png", "side": "tron.png"},
            "cells": {"top": [0, 0, 800, 764], "side": [800, 0, 1259, 764]},
            "label": {"en": "Tron-inspired", "fr": "Inspiré Tron"},
            "dock": "styled",
            "mapMaxSize": 384,
            "wheelsOverShell": {"top": false, "side": true},
            "paint": {
                "masks": {"top": "tron-top-paint.svg", "side": "tron-side-paint.svg"},
                "suggestedColour": "#3f8e79"
            }
        },
        "rounded": {
            "size": [1774, 887],
            "top": [75, 63, 620, 788],
            "side": [911, 339, 825, 284],
            "files": {
                "top": "rounded.png",
                "side": "rounded.png"
            },
            "label": {
                "en": "Rounded",
                "fr": "Arrondi"
            },
            "dock": "styled",
            "mapMaxSize": 384,
            "wheelsOverShell": {
                "top": false,
                "side": false
            },
            "cells": {
                "top": [0, 0, 768, 887],
                "side": [768, 0, 1006, 887]
            },
            "paint": {
                "masks": {
                    "top": "rounded-top-paint.svg",
                    "side": "rounded-side-paint.svg"
                },
                "suggestedColour": "#3f8e79"
            }
        },
        "sculpted": {
            "size": [1774, 887],
            "top": [106, 32, 630, 829],
            "side": [795, 302, 931, 338],
            "files": {
                "top": "sculpted.png",
                "side": "sculpted.png"
            },
            "label": {
                "en": "Sculpted",
                "fr": "Sculpté"
            },
            "dock": "styled",
            "mapMaxSize": 384,
            "wheelsOverShell": {
                "top": false,
                "side": false
            },
            "cells": {
                "top": [0, 0, 768, 887],
                "side": [768, 0, 1006, 887]
            },
            "paint": {
                "masks": {
                    "top": "sculpted-top-paint.svg",
                    "side": "sculpted-side-paint.svg"
                },
                "suggestedColour": "#3f8e79"
            }
        },
        "utility": {
            "size": [1774, 887],
            "top": [117, 48, 607, 816],
            "side": [831, 356, 874, 261],
            "files": {
                "top": "utility.png",
                "side": "utility.png"
            },
            "label": {
                "en": "Utility",
                "fr": "Utilitaire"
            },
            "dock": "styled",
            "mapMaxSize": 384,
            "wheelsOverShell": {
                "top": false,
                "side": false
            },
            "cells": {
                "top": [0, 0, 768, 887],
                "side": [768, 0, 1006, 887]
            },
            "paint": {
                "masks": {
                    "top": "utility-top-paint.svg",
                    "side": "utility-side-paint.svg"
                },
                "suggestedColour": "#3f8e79"
            }
        },
        "yardforce": {
            "size": [1774, 887],
            "top": [113, 29, 629, 818],
            "side": [852, 274, 857, 371],
            "files": {
                "top": "yardforce.png",
                "side": "yardforce.png"
            },
            "label": {
                "en": "Yardforce-inspired",
                "fr": "Inspiré Yardforce"
            },
            "dock": "styled",
            "mapMaxSize": 384,
            "wheelsOverShell": {
                "top": false,
                "side": false
            },
            "cells": {
                "top": [0, 0, 768, 887],
                "side": [768, 0, 1006, 887]
            },
            "paint": {
                "masks": {
                    "top": "yardforce-top-paint.svg",
                    "side": "yardforce-side-paint.svg"
                },
                "suggestedColour": "#e97822"
            }
        },
        "rm1000": {
            "files": {
                "top": "rm1000.png",
                "side": "rm1000-side.png"
            },
            "size": [1774, 887],
            "top": [73, 48, 624, 804],
            "side": [44, 162, 1702, 622],
            "label": {
                "en": "RM1000-inspired",
                "fr": "Inspiré RM1000"
            },
            "dock": "styled",
            "mapMaxSize": 384,
            "wheelsOverShell": {
                "top": false,
                "side": false
            },
            "cells": {
                "top": [0, 0, 740, 887],
                "side": [0, 0, 1774, 887]
            },
            "paint": {
                "masks": {
                    "top": "rm1000-top-paint.svg",
                    "side": "rm1000-side-paint.svg"
                },
                "suggestedColour": "#167bc1"
            }
        },
        "worx": {
            "files": {
                "top": "worx.png",
                "side": "worx-side.png"
            },
            "size": [1774, 887],
            "sizes": {
                "side": [1254, 1254]
            },
            "top": [102, 67, 586, 698],
            "side": [44, 487, 793, 295],
            "frames": {
                "side": [44, 487, 1167, 295]
            },
            "label": {
                "en": "Worx-inspired",
                "fr": "Inspiré Worx"
            },
            "dock": "worx",
            "mapMaxSize": 384,
            "wheelsOverShell": {
                "top": false,
                "side": true
            },
            "cells": {
                "top": [0, 0, 768, 887],
                "side": [0, 0, 1254, 1254]
            },
            "paint": {
                "masks": {
                    "top": "worx-top-paint.svg",
                    "side": "worx-side-paint.svg"
                },
                "suggestedColour": "#e97822"
            }
        }
    },
    "docks": {
        "standard": {
            "part": "dock",
            "height": 0.15,
            "origin": "rearAxle",
            "offset": {
                "forward": 0.22,
                "left": 0
            },
            "topAnchor": [0.5, 0.5],
            "sideAnchor": [0.5, 1],
            "appearanceId": "styled",
            "label": {
                "en": "Standard dock",
                "fr": "Station standard"
            }
        },
        "worx": {
            "part": "dock-worx",
            "height": 0.14,
            "origin": "chassisCenter",
            "offset": {
                "forward": 0,
                "left": 0
            },
            "topAnchor": [0.42718446601941745, 0.5],
            "sideAnchor": [0.5, 1],
            "appearanceId": "worx",
            "label": {
                "en": "Side-charging dock",
                "fr": "Station latérale"
            }
        }
    },
    "parts": {
        "blade": {
            "size": [1774, 887],
            "top": {
                "crop": [40, 31, 787, 751],
                "reference": [10, 30, 844, 844]
            },
            "side": {
                "crop": [896, 371, 853, 142],
                "reference": [896, 435, 853, 40]
            },
            "files": {
                "top": "blade.png",
                "side": "blade.png"
            },
            "mapMaxSize": 256
        },
        "dock-worx": {
            "size": [1774, 887],
            "sizes": {"top": [1774, 887], "side": [1984, 793]},
            "top": {
                "crop": [47, 53, 721, 772]
            },
            "side": {
                "crop": [100, 282, 1784, 332]
            },
            "files": {
                "top": "dock-worx.png",
                "side": "dock-worx-side.png"
            },
            "mapMaxSize": 384
        },
        "dock": {
            "size": [1536, 1024],
            "top": {
                "crop": [33, 74, 560, 806]
            },
            "side": {
                "crop": [635, 412, 883, 250]
            },
            "files": {
                "top": "dock.png",
                "side": "dock.png"
            },
            "mapMaxSize": 384
        },
        "drive-wheel": {
            "size": [1774, 887],
            "top": {
                "crop": [258, 103, 251, 682]
            },
            "side": {
                "crop": [1058, 138, 629, 620]
            },
            "files": {
                "top": "drive-wheel.png",
                "side": "drive-wheel.png"
            },
            "mapMaxSize": 128
        },
        "caster": {
            "size": [1536, 1024],
            "top": {
                "crop": [142, 56, 384, 909],
                "reference": [192, 56, 282, 909]
            },
            "side": {
                "crop": [718, 115, 724, 803],
                "reference": [750, 240, 690, 677]
            },
            "files": {
                "top": "caster.png",
                "side": "caster.png"
            },
            "mapMaxSize": 128
        },
        "gps": {
            "size": [1774, 887],
            "top": {
                "crop": [152, 128, 627, 611]
            },
            "side": {
                "crop": [995, 364, 675, 166]
            },
            "files": {
                "top": "gps.png",
                "side": "gps.png"
            },
            "mapMaxSize": 128
        },
        "lidar": {
            "size": [1983, 793],
            "top": {
                "crop": [176, 112, 529, 544]
            },
            "side": {
                "crop": [1257, 219, 563, 337]
            },
            "files": {
                "top": "lidar.png",
                "side": "lidar.png"
            },
            "mapMaxSize": 128
        },
        "imu": {
            "size": [1983, 793],
            "top": {
                "crop": [263, 178, 405, 388]
            },
            "side": {
                "crop": [1263, 354, 457, 77]
            },
            "files": {
                "top": "imu.png",
                "side": "imu.png"
            },
            "mapMaxSize": 128
        }
    },
    "legacyDocks": {
        "marker": {
            "id": "marker",
            "labelKey": "mapToolbar.dockAppearanceMarker"
        },
        "generic": {
            "id": "generic",
            "labelKey": "mapToolbar.dockAppearanceGeneric",
            "foregroundClipPath": "polygon(43% 78%, 57% 78%, 68% 83%, 68% 92%, 59% 96%, 41% 96%, 32% 92%, 32% 83%)",
            "image": {
                "src": "/assets/robots/generic/dock.webp",
                "altKey": "mapToolbar.dockAppearanceGenericAlt",
                "visibleLengthM": 0.63,
                "visibleLengthFraction": 0.951,
                "visibleWidthM": 0.46,
                "visibleWidthFraction": 0.678,
                "poseAnchor": {
                    "x": 0.5,
                    "y": 0.16
                },
                "headingOffsetRad": 3.141592653589793
            }
        },
        "biltema-rm1000": {
            "id": "biltema-rm1000",
            "labelKey": "mapToolbar.dockAppearanceBiltemaRm1000",
            "foregroundClipPath": "polygon(43% 78%, 57% 78%, 68% 83%, 68% 92%, 59% 96%, 41% 96%, 32% 92%, 32% 83%)",
            "image": {
                "src": "/assets/robots/biltema-rm1000/dock.webp",
                "altKey": "mapToolbar.dockAppearanceBiltemaRm1000Alt",
                "visibleLengthM": 0.63,
                "visibleLengthFraction": 0.944,
                "visibleWidthM": 0.46,
                "visibleWidthFraction": 0.667,
                "poseAnchor": {
                    "x": 0.5,
                    "y": 0.16
                },
                "headingOffsetRad": 3.141592653589793
            }
        }
    }
} as const;
