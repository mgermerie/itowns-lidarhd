import * as THREE from 'three';
import PointCloudNode from 'Core/PointCloudNode';
import CopcNode from 'Core/CopcNode';
import EntwinePointTileNode from 'Core/EntwinePointTileNode';
import PointCloudLayer from 'Layer/PointCloudLayer';


// PointCLoudLayer functions
const point = new THREE.Vector3();
const bboxMesh = new THREE.Mesh();
const box3 = new THREE.Box3();
bboxMesh.geometry.boundingBox = box3;

function computeSSEPerspective(context, pointSize, pointSpacing, distance) {
    if (distance <= 0) {
        return Infinity;
    }
    // Estimate the onscreen distance between 2 points
    const onScreenSpacing = context.camera.preSSE * pointSpacing / distance;
    // [  P1  ]--------------[   P2   ]
    //     <--------------------->      = pointsSpacing (in world coordinates)
    //                                  ~ onScreenSpacing (in pixels)
    // <------>                         = pointSize (in pixels)
    return Math.max(0.0, onScreenSpacing - pointSize);
}

function computeSSEOrthographic(context, pointSize, pointSpacing) {
    // Given an identity view matrix, project pointSpacing from world space to
    // clip space. v' = vVP = vP
    const v = new THREE.Vector4(pointSpacing);
    v.applyMatrix4(context.camera.camera3D.projectionMatrix);

    // We map v' to the screen space and calculate the distance to the origin.
    const dx = v.x * 0.5 * context.camera.width;
    const dy = v.y * 0.5 * context.camera.height;
    const distance = Math.sqrt(dx * dx + dy * dy);

    return Math.max(0.0, distance - pointSize);
}

function computeScreenSpaceError(context, pointSize, pointSpacing, distance) {
    if (context.camera.camera3D.isOrthographicCamera) {
        return computeSSEOrthographic(context, pointSize, pointSpacing);
    }

    return computeSSEPerspective(context, pointSize, pointSpacing, distance);
}

function markForDeletion(elt) {
    if (elt.obj) {
        elt.obj.visible = false;
        if (__DEBUG__) {
            if (elt.obj.boxHelper) {
                elt.obj.boxHelper.visible = false;
            }
        }
    }

    if (!elt.notVisibleSince) {
        elt.notVisibleSince = Date.now();
        // Set .sse to an invalid value
        elt.sse = -1;
    }
    for (const child of elt.children) {
        markForDeletion(child);
    }
}

/**
 * A layer for [Cloud Optimised Point Cloud](https://copc.io) (COPC) datasets.
 * See {@link PointCloudLayer} class for documentation on base properties.
 *
 * @extends {PointCloudLayer}
 *
 * @example
 * // Create a new VPC layer
 * const vpcSource = new VpcSource({
 *     url: 'https://data.geopf.fr/chunk/telechargement/download/lidarhd_fxx_ept/vpc/index.vpc',
 * });
 *
 * const vpcLayer = new VpcLayer('VPC', {
 *     source: vpcSource,
 * });
 *
 * View.prototype.addLayer.call(view, vpcLayer);
 */
class VpcLayer extends PointCloudLayer {
    /**
     * @param {string} id - Unique id of the layer.
     * @param {Object} config - See {@link PointCloudLayer} for base pointcloud
     * options.
     */
    constructor(id, config) {
        super(id, config);

        /**
         * @type {boolean}
         * @readonly
         */
        this.isVpcLayer = true;

        this.roots = [];
        this.spacing = [];
        this.scale = new THREE.Vector3(1.0, 1.0, 1.0);
        this.offset = new THREE.Vector3(0.0, 0.0, 0.0);

        const zmins = [];
        const zmaxs = [];

        // const resolve = this.addInitializationStep();
        const resolve = res => res;

        this.loadOctrees = [];
        this.whenReady = this.source.whenReady.then((/** @type {VpcSource} */ sources) => {
            sources.forEach((source, i) => {
                const boundsConforming = source.boundsConforming;

                this.zmin = source.boundsConforming[2];
                this.zmax = source.boundsConforming[5];
                zmins.push(this.zmin);
                zmaxs.push(this.zmax);

                // const bbox = new THREE.Box3().setFromArray(boundsConforming);
                this.root = new PointCloudNode(0, this);
                this.setRootBbox(boundsConforming.slice(0, 3), boundsConforming.slice(3, 6));
                const root = {
                    voxelOBB: this.root.voxelOBB.clone(),
                    clampOBB: this.root.clampOBB.clone(),
                    children: [],
                    sId: i,
                };
                root.voxelOBB.matrixWorldInverse = this.root.voxelOBB.matrixWorldInverse;
                root.clampOBB.matrixWorldInverse = this.root.clampOBB.matrixWorldInverse;

                const promise =
                    this.source.sources[i].whenReady.then((src) => {
                        if (this.source.sources[i].isCopcSource) {
                            const { cube, rootHierarchyPage } = src.info;
                            const { pageOffset, pageLength } = rootHierarchyPage;

                            this.spacing.push(src.info.spacing);

                            const root = new CopcNode(0, 0, 0, 0, pageOffset, pageLength, this, -1, i);
                            // root.bbox.min.fromArray(cube, 0);
                            // root.bbox.max.fromArray(cube, 3);
                            this.setRootBbox(cube.slice(0, 3), cube.slice(3, 6));
                            root.voxelOBB = this.root.voxelOBB.clone();
                            root.clampOBB = this.root.clampOBB.clone();
                            root.voxelOBB.matrixWorldInverse = this.root.voxelOBB.matrixWorldInverse;
                            root.clampOBB.matrixWorldInverse = this.root.clampOBB.matrixWorldInverse;
                            this.roots[i] = root;

                            return root.loadOctree().then(res => resolve(res));
                        } else {
                            const spacing = (Math.abs(src.bounds[3] - src.bounds[0])
                                + Math.abs(src.bounds[4] - src.bounds[1])) / (2 * src.span);
                            this.spacing.push(spacing);

                            const root = new EntwinePointTileNode(0, 0, 0, 0, this, -1, i);
                            // root.bbox.min.fromArray(src.boundsConforming, 0);
                            // root.bbox.max.fromArray(src.boundsConforming, 3);
                            this.setRootBbox(src.bounds.slice(0, 3), src.bounds.slice(3, 6));
                            root.voxelOBB = this.root.voxelOBB.clone();
                            root.clampOBB = this.root.clampOBB.clone();
                            root.voxelOBB.matrixWorldInverse = this.root.voxelOBB.matrixWorldInverse;
                            root.clampOBB.matrixWorldInverse = this.root.clampOBB.matrixWorldInverse;

                            this.roots[i] = root;

                            return root.loadOctree().then(res => resolve(res));
                        }
                    });
                this.loadOctrees.push(promise);

                root.load = () => this.loadOctrees[i].then(res => res.load());
                this.roots.push(root);
            });

            this.zmin = Math.min(...zmins);
            this.zmax = Math.max(...zmaxs);
            this.setElevationRange();

            this.ready = true;

            return this.loadOctrees;
        });
    }

    // adapted from PointCloudLayer, return are differents
    preUpdate(context, changeSources) {
        // See https://cesiumjs.org/hosted-apps/massiveworlds/downloads/Ring/WorldScaleTerrainRendering.pptx
        // slide 17
        context.camera.preSSE =
            context.camera.height /
                (2 * Math.tan(THREE.MathUtils.degToRad(context.camera.camera3D.fov) * 0.5));

        if (this.material) {
            this.material.visible = this.visible;
            this.material.opacity = this.opacity;
            this.material.depthWrite = false;
            this.material.size = this.pointSize;
            this.material.scale = context.camera.preSSE;
            if (this.material.updateUniforms) {
                this.material.updateUniforms();
            }
        }

        // lookup lowest common ancestor of changeSources
        let commonAncestor;
        for (const source of changeSources.values()) {
            if (source.isCamera || source == this) {
                // if the change is caused by a camera move, no need to bother
                // to find common ancestor: we need to update the whole tree:
                // some invisible tiles may now be visible
                return this.roots;
            }
            if (source.obj === undefined) {
                continue;
            }
            // filter sources that belong to our layer
            if (source.obj.isPoints && source.obj.layer == this) {
                if (!commonAncestor) {
                    commonAncestor = source;
                } else {
                    commonAncestor = source.findCommonAncestor(commonAncestor);

                    if (!commonAncestor) {
                        return this.roots;
                    }
                }
            }
        }

        if (commonAncestor) {
            return [commonAncestor];
        }

        // Start updating from hierarchy root
        return this.roots;
    }

    // PointCloudLayer.update separate in 2 parts: update and subUpdate
    subUpdate(elt, context, layer, bbox, matrixWorldInverse) {
        elt.notVisibleSince = undefined;

        point.copy(context.camera.camera3D.position).applyMatrix4(matrixWorldInverse);
        const distanceToCamera = bbox.distanceToPoint(point);

        if (elt.numPoints !== 0) {
            if (elt.obj) {
                elt.obj.visible = true;
            } else if (!elt.promise) {
                const distance = Math.max(0.001, distanceToCamera);
                // Increase priority of nearest node
                const priority = computeScreenSpaceError(context, layer.pointSize, layer.spacing[elt.sId], elt, distance) / distance;
                elt.promise = context.scheduler.execute({
                    layer,
                    requester: elt,
                    view: context.view,
                    priority,
                    redraw: true,
                    earlyDropFunction: cmd => !cmd.requester.visible || !this.visible,
                }).then((pts) => {
                    elt.obj = pts;

                    // make sure to add it here, otherwise it might never
                    // be added nor cleaned
                    this.group.add(elt.obj);
                    elt.obj.updateMatrixWorld(true);
                }).catch((err) => {
                    if (!err.isCancelledCommandException) {
                        return err;
                    }
                }).finally(() => {
                    elt.promise = null;
                });
            }
        }

        if (elt.children && elt.children.length) {
            elt.sse = computeScreenSpaceError(context, layer.pointSize, 0.5 * layer.spacing[elt.sId], distanceToCamera) / this.sseThreshold;
            if (elt.sse >= 1) {
                return elt.children;
            } else {
                for (const child of elt.children) {
                    markForDeletion(child);
                }
            }
        }
    }

    update(context, layer, elt) {
        elt.visible = false;

        if (this.octreeDepthLimit >= 0 && this.octreeDepthLimit < elt.depth) {
            markForDeletion(elt);
            return [];
        }

        // get object on which to measure distance
        let bbox;
        let matrixWorld;
        let matrixWorldInverse;

        if (elt.obj) {
            const obj = elt.obj;
            bbox = obj.geometry.boundingBox;
            matrixWorld = obj.matrixWorld;
            matrixWorldInverse = obj.matrixWorldInverse;
        } else {
            bbox = elt.clampOBB.box3D;
            matrixWorld = elt.clampOBB.matrixWorld;
            matrixWorldInverse = elt.clampOBB.matrixWorldInverse;
        }

        elt.visible = context.camera.isBox3Visible(bbox, matrixWorld);

        if (!elt.visible) {
            markForDeletion(elt);
            return [];
        }

        if (!(elt.isCopcNode || elt.isEntwinePointTileNode)) {
            layer.source.load(elt.sId);
            layer.loadOctrees[elt.sId]
                .then(() => {
                    elt = this.roots[elt.sId];
                    elt.visible = true;
                    return this.subUpdate(elt, context, layer, bbox, matrixWorldInverse);
                });
        } else {
            return this.subUpdate(elt, context, layer, bbox, matrixWorldInverse);
        }
    }

    /*
    postUpdate() {
    }
    */
}

export default VpcLayer;
