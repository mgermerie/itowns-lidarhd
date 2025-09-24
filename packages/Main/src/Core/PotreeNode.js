import * as THREE from 'three';
import PointCloudNode from 'Core/PointCloudNode';

// Create an A(xis)A(ligned)B(ounding)B(ox) for the child `childIndex` of one aabb.
// (PotreeConverter protocol builds implicit octree hierarchy by applying the same
// subdivision algo recursively)
const dHalfLength = new THREE.Vector3();

class PotreeNode extends PointCloudNode {
    constructor(numPoints = 0, childrenBitField = 0, layer) {
        super(numPoints, layer);
        this.childrenBitField = childrenBitField;

        this.depth = 0;

        this.hierarchyKey = 'r';

        this.baseurl = layer.source.baseurl;
    }

    get octreeIsLoaded() {
        return !(this.childrenBitField && this.children.length === 0);
    }

    get url() {
        return `${this.baseurl}/${this.hierarchyKey}.${this.layer.source.extension}`;
    }

    get id() {
        return this.hierarchyKey;
    }

    add(node, indexChild) {
        node.hierarchyKey = this.hierarchyKey + indexChild;
        node.depth = this.depth + 1;
        super.add(node, indexChild);
    }

    createChildAABB(childNode, childIndex) {
        childNode.voxelOBB.copy(this.voxelOBB);
        const voxelBBox = this.voxelOBB.box3D;
        const childVoxelBBox = childNode.voxelOBB.box3D;

        // Code inspired from potree
        childVoxelBBox.copy(voxelBBox);
        voxelBBox.getCenter(childVoxelBBox.max);
        dHalfLength.copy(childVoxelBBox.max).sub(voxelBBox.min);

        if (childIndex === 1) {
            childVoxelBBox.min.z += dHalfLength.z;
            childVoxelBBox.max.z += dHalfLength.z;
        } else if (childIndex === 3) {
            childVoxelBBox.min.z += dHalfLength.z;
            childVoxelBBox.max.z += dHalfLength.z;
            childVoxelBBox.min.y += dHalfLength.y;
            childVoxelBBox.max.y += dHalfLength.y;
        } else if (childIndex === 0) {
            //
        } else if (childIndex === 2) {
            childVoxelBBox.min.y += dHalfLength.y;
            childVoxelBBox.max.y += dHalfLength.y;
        } else if (childIndex === 5) {
            childVoxelBBox.min.z += dHalfLength.z;
            childVoxelBBox.max.z += dHalfLength.z;
            childVoxelBBox.min.x += dHalfLength.x;
            childVoxelBBox.max.x += dHalfLength.x;
        } else if (childIndex === 7) {
            childVoxelBBox.min.add(dHalfLength);
            childVoxelBBox.max.add(dHalfLength);
        } else if (childIndex === 4) {
            childVoxelBBox.min.x += dHalfLength.x;
            childVoxelBBox.max.x += dHalfLength.x;
        } else if (childIndex === 6) {
            childVoxelBBox.min.y += dHalfLength.y;
            childVoxelBBox.max.y += dHalfLength.y;
            childVoxelBBox.min.x += dHalfLength.x;
            childVoxelBBox.max.x += dHalfLength.x;
        }

        childNode.clampOBB.copy(childNode.voxelOBB);

        const childClampBBox = childNode.clampOBB.box3D;

        if (childClampBBox.min.z < this.layer.zmax) {
            childClampBBox.max.z = Math.min(childClampBBox.max.z, this.layer.zmax);
        }
        if (childClampBBox.max.z > this.layer.zmin) {
            childClampBBox.min.z = Math.max(childClampBBox.min.z, this.layer.zmin);
        }

        childNode.voxelOBB.matrixWorldInverse = this.voxelOBB.matrixWorldInverse;
        childNode.clampOBB.matrixWorldInverse = this.clampOBB.matrixWorldInverse;
    }

    load() {
        // Query octree/HRC if we don't have children potreeNode yet.
        if (!this.octreeIsLoaded) {
            this.loadOctree();
        }
        return super.load();
    }

    // See if to keep for Potree1
    // getCenter() {
    //     // With the potree format the node data are already encoded using the min corner of the bbox as origin.
    //     // Linked with the reprojection of points, we might need to change that to the real center but it
    //     // would need to make changes in the parser.
    //     return this.voxelOBB.box3D.min;
    // }

    loadOctree() {
        const octreeUrl = `${this.baseurl}/${this.hierarchyKey}.${this.layer.source.extensionOctree}`;
        return this.layer.source.fetcher(octreeUrl, this.layer.source.networkOptions).then((blob) => {
            const view = new DataView(blob);
            const stack = [];
            let offset = 0;

            this.childrenBitField = view.getUint8(0); offset += 1;
            this.numPoints = view.getUint32(1, true); offset += 4;

            stack.push(this);

            while (stack.length && offset < blob.byteLength) {
                const snode = stack.shift();
                // look up 8 children
                for (let indexChild = 0; indexChild < 8; indexChild++) {
                    // does snode have a #indexChild child ?
                    if (snode.childrenBitField & (1 << indexChild) && (offset + 5) <= blob.byteLength) {
                        const childrenBitField = view.getUint8(offset); offset += 1;
                        const numPoints = view.getUint32(offset, true) || this.numPoints; offset += 4;
                        const child = new PotreeNode(numPoints, childrenBitField, this.layer);
                        snode.add(child, indexChild);
                        if ((child.id.length % this.layer.hierarchyStepSize) == 0) {
                            child.baseurl = `${this.baseurl}/${child.id}`;
                        } else {
                            child.baseurl = this.baseurl;
                        }
                        stack.push(child);
                    }
                }
            }
        });
    }
}

export default PotreeNode;
