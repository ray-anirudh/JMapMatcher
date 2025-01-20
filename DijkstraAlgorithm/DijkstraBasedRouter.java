/**
 * Author: Anirudh Ray
 * Institution: Professorship of Traffic Engineering and Control, Technical University of Munich
 * Department: Mobility Systems Engineering, School of Engineering and Design
 * E-mail Address: Anirudh.Ray@tum.de
 * Purpose: Component of a Java-based multi-modal routing algorithm, built using RAPTOR, Dijkstra-algorithm, and
 * KD-Trees
 */

package DijkstraAlgorithm;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.TreeMap;

public class DijkstraBasedRouter {
    private final double ROAD_VS_AERIAL_DISTANCE_FACTOR = 1.45;
    /* Refer to: https://link.springer.com/chapter/10.1007/978-3-030-12381-9_12 or
    https://ctl.mit.edu/pub/workingpaper/quantifying-impact-urban-road-networks-efficiency-local-trips for more details
    */

    private ArrayList<Node> nodeListForTraversal = new ArrayList<>();   // For sequential path tracking
    private boolean distanceFixingFlag = false;

    public double findShortestDrivingPathLengthM(long originNodeId, long destinationNodeId,
                                                 LinkedHashMap<Long, Node> nodes, LinkedHashMap<Long, Link> links) {
        double travelDistanceM;

        // Initialize variables and collections for iterations
        TreeMap<Double, Long> visitedNodes = new TreeMap<>();
        visitedNodes.put(0D, originNodeId);    // Path length for origin node is treated as zero
        TreeMap<Double, Long> nodesUnderEvaluation = new TreeMap<>();
        HashSet<Long> traversedLinksIds = new HashSet<>();

        // Execute Dijkstra's algorithm
        while (!(visitedNodes.lastEntry().getValue().equals(destinationNodeId))) {
            long currentNodeId = visitedNodes.lastEntry().getValue();
            double currentNodePathLengthM = visitedNodes.lastEntry().getKey();

            for (long linkIdUnderConsideration : nodes.get(currentNodeId).getLinkIdList()) {
                if (!traversedLinksIds.contains(linkIdUnderConsideration)) {
                    Link linkUnderConsideration = links.get(linkIdUnderConsideration);
                    double pathLengthToOtherNodeM = currentNodePathLengthM + linkUnderConsideration.
                            getlinkLengthM();
                    long otherNodeId = (linkUnderConsideration.getFirstNodeId() == currentNodeId) ?
                            linkUnderConsideration.getSecondNodeId() : linkUnderConsideration.getFirstNodeId();
                    /* Debugging statements:
                    if(!((currentNodeId == linkUnderConsideration.getFirstNodeId()) ||
                        (currentNodeId == linkUnderConsideration.getSecondNodeId()))) {
                        System.out.println("Failure!");
                    }
                    */

                    // Update the collections of nodes yet to be relaxed and of links whose travel times are known
                    nodesUnderEvaluation.put(pathLengthToOtherNodeM, otherNodeId);
                    traversedLinksIds.add(linkIdUnderConsideration);

                    // Set predecessor node for all outgoing nodes
                    nodes.get(otherNodeId).setPredecessorNodeId(currentNodeId);
                }
            }

            // Add the cheapest node to the collection of visited nodes, thereby removing it from under observation
            if (!nodesUnderEvaluation.isEmpty()) {
                /* Debugging statement:
                System.out.println("Using Dijkstra's algorithm.");
                */
                visitedNodes.put(nodesUnderEvaluation.firstKey(), nodesUnderEvaluation.firstEntry().getValue());
                nodesUnderEvaluation.remove(nodesUnderEvaluation.firstKey());
            } else {
                // System.out.println("Used distance-fixing.");
                travelDistanceM = nodes.get(originNodeId).equiRectangularDistanceTo(nodes.get(destinationNodeId).
                        getNodeLongitude(), nodes.get(destinationNodeId).getNodeLatitude()) *
                        ROAD_VS_AERIAL_DISTANCE_FACTOR;
                this.distanceFixingFlag = true;
                return travelDistanceM;
            }
        }

        // Return the travel distance in meters
        travelDistanceM = visitedNodes.lastEntry().getKey();
        return travelDistanceM;
    }

    public ArrayList<Node> createNodeListForTraversal(long originNodeId, long destinationNodeId,
                                                      LinkedHashMap<Long, Node> nodes) {
        if (!this.distanceFixingFlag) {
            long currentNodeId = destinationNodeId;
            while (currentNodeId != originNodeId) {
                this.nodeListForTraversal.add(nodes.get(currentNodeId));
                currentNodeId = nodes.get(currentNodeId).getPredecessorNodeId();
            }
        } else {
            this.nodeListForTraversal.add(nodes.get(destinationNodeId));
        }

        return this.nodeListForTraversal;
    }

}