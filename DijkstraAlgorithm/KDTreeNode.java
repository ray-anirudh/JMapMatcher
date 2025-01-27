/**
 * Author: Anirudh Ray
 * Institution: Professorship of Traffic Engineering and Control, Technical University of Munich
 * Department: Mobility Systems Engineering, School of Engineering and Design
 * E-mail Address: Anirudh.Ray@tum.de
 * Purpose: Component of a Java-based multi-modal routing algorithm, built using RAPTOR, Dijkstra-algorithm, and
 * KD-Trees
 */

package DijkstraAlgorithm;

public class KDTreeNode {   // Use for node-based KD-Trees
    private Node node;  // KD-Tree nodes are modelled on top of network nodes
    private KDTreeNode left;
    private KDTreeNode right;

    KDTreeNode(Node node) {
        this.node = node;
        this.left = this.right = null;
    }

    public void setLeft(KDTreeNode left) {
        this.left = left;
    }

    public void setRight(KDTreeNode right) {
        this.right = right;
    }

    public Node getNode() {
        return this.node;
    }

    public KDTreeNode getLeft() {
        return this.left;
    }

    public KDTreeNode getRight() {
        return this.right;
    }
}