/**
 * Authors: Anirudh Ray and Matthias Länger
 * Institution: Professorship of Travel Behaviour, Technical University of Munich
 * Department: Mobility Systems Engineering, School of Engineering and Design
 * E-mail Address: Anirudh.Ray@tum.de
 * Purpose: Component of a Java-based map-matching algorithm and GPS data analyser
 */

import DijkstraAlgorithm.*;
import GPSDataManager.GPSNode;
import GPSDataManager.GPSNodeReaderWriter;
import KDTreeManager.KDTreeBuilderSearcher;
import OSMDataManager.NetworkNode;
import OSMDataManager.NetworkNodeReader;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class RouteAligningAlgorithmNNFinderRaw {
    public static void main(String[] args) {
        String gPSNodesFilePath = "E:/anirudh/CargoBikeProject/RawData/GPSData/2024-09-04_GPSrecords_edited.csv";
        String hMMNodesFilePath = "E:/anirudh/CargoBikeProject/RawData/Osmium/" +
                "OSMRoadNetworkForCargoBikes1mPointsHMM.csv";
        // String dijkstraLinksFilePath = "E:/anirudh/CargoBikeProject/Results/dijkstraLinks.csv";
        // String dijkstraNodesFilePath = "E:/anirudh/CargoBikeProject/Results/dijkstraNodes.csv";
        int numberNeighbouringNetworkNodesCompared = 1;
        String routingNetworkFilePath = "E:/anirudh/CargoBikeProject/RawData/Osmium/" +
                "planet_11.229,48_11.914,48.278.osm.opl";
        String snappedGPSNodesFilePath = "E:/anirudh/CargoBikeProject/Results/gPSNodesSnappedNNFinder.csv";
        String pathNodesFilePath = "E:/anirudh/CargoBikeProject/Results/pathNodesNNFinder.csv";
        String shortestPathNodesFilePath = "E:/anirudh/CargoBikeProject/Results/shortestPathNodesNNFinder.csv";

        // Load all GPS nodes to be snapped
        GPSNodeReaderWriter gpsNodeReaderWriter = new GPSNodeReaderWriter();
        gpsNodeReaderWriter.readGPSNodes(gPSNodesFilePath);
        LinkedHashMap<Long, GPSNode> gPSNodes = gpsNodeReaderWriter.getGPSNodes();
        // System.out.println(gPSNodes.get(40L).toString());    // Debugging statement

        // Load all network nodes to be snapped to
        NetworkNodeReader networkNodeReader = new NetworkNodeReader();
        networkNodeReader.readNetworkNodes(hMMNodesFilePath);
        networkNodeReader.buildConnectivityMatrix();
        LinkedHashMap<Long, NetworkNode> networkNodes = networkNodeReader.getNetworkNodes();
        LinkedHashMap<Long, ArrayList<Long>> connectivityMatrix = networkNodeReader.getConnectivityMatrix();
        // for (NetworkNode networkNode : networkNodes.values()) {System.out.println(networkNode.toString());}

        // Load the graph structures to be used for routing and snapping
        OSMDataReaderWriter osmDataReaderWriter = new OSMDataReaderWriter();
        osmDataReaderWriter.readAndFilterOsmLinks(routingNetworkFilePath);
        osmDataReaderWriter.removeCircularLinks();
        osmDataReaderWriter.readAndFilterOsmNodes(routingNetworkFilePath);
        osmDataReaderWriter.associateLinksWithNode();
        osmDataReaderWriter.calculateLinkLengthsM();
        LinkedHashMap<Long, Node> nodes = osmDataReaderWriter.getNodes();
        LinkedHashMap<Long, Link> links = osmDataReaderWriter.getLinks();
        Node[] nodesForSnapping = nodes.values().toArray(new Node[0]);
        KDTreeForNodes kDTreeForDijkstra = new KDTreeForNodes();
        kDTreeForDijkstra.buildNodeBasedKDTree(nodesForSnapping);
        // osmDataReaderWriter.writeDijkstraLinks(dijkstraLinksFilePath);
        // osmDataReaderWriter.writeDijkstraNodes(dijkstraNodesFilePath);
        LinkedHashMap<Integer, Node> gPSPointPathSequenceNodes = new LinkedHashMap<>();
        LinkedHashMap<Integer, Node> gPSPointIsStopPathSequenceNodes = new LinkedHashMap<>();

        // Find the candidate network nodes for each GPS node, i.e., carry out map matching
        KDTreeBuilderSearcher kdTreeBuilderSearcher = new KDTreeBuilderSearcher();
        NetworkNode[] kDTreeForHMM = networkNodes.values().toArray(new NetworkNode[0]);
        kdTreeBuilderSearcher.buildNodeBasedKDTree(kDTreeForHMM);

        NetworkNode previousState = null;
        for (GPSNode gPSNode : gPSNodes.values()) {
            // System.out.println("GPS node ID: " + gPSNode.getGPSNodeId());    // Debugging statement
            List<NetworkNode> candidateStateNodes = kdTreeBuilderSearcher.findNearestNodes(gPSNode.
                    getGPSNodeLongitude(), gPSNode.getGPSNodeLatitude(), numberNeighbouringNetworkNodesCompared);

            NetworkNode nearestNetworkNode = candidateStateNodes.get(0);
            gPSNode.setMatchedNodeLongitude(nearestNetworkNode.getNetworkNodeLongitude());
            gPSNode.setMatchedNodeLatitude(nearestNetworkNode.getNetworkNodeLatitude());
            gPSNode.setMatchedNodeOsmWayId(nearestNetworkNode.getRoadOsmId());
            gPSNode.setJointProbability(1);
            gPSNode.setMatchedNodeElevationM(0);
        }

        // Route between different network nodes (states) and feed the sequencing arraylists to the master map
        GPSNode previousGPSNode = gPSNodes.get(1L);
        gPSNodes.get(1L).setDistanceFromPreviousGPSPointM(0);

        Node previousPathNode = kDTreeForDijkstra.findNearestNode(gPSNodes.get(1L).getMatchedNodeLongitude(),
                gPSNodes.get(1L).getMatchedNodeLatitude());// First record handling
        int pathNodeFeatureId = 1;
        long osmWayIdToAscribe = 0L;
        double distanceFromPreviousFeatureM = 0;

        try {
            BufferedWriter pathNodeWriter = new BufferedWriter(new FileWriter(pathNodesFilePath));
            pathNodeWriter.write("FeatureId,OSMNodeId,FeatureLongitude,FeatureLatitude,FromGPSPointId," +
                    "ToGPSPointId,FromGPSPointLongitude,FromGPSPointLatitude,ToGPSPointLongitude," +
                    "ToGPSPointLatitude,FromGPSPointDateTime,ToGPSPointDateTime,FromGPSPointAtDepot," +
                    "FromGPSPointAtDepotLag,FromGPSPointAtDepotNext,FromGPSPointHubStay,FromGPSPointHubStart," +
                    "FromGPSPointHubStop,FromGPSPointTourNumber,FromGPSPointIsStop,OSMWayId," +
                    "DistanceFromPreviousFeature,IsIntersection\n");

            for (GPSNode gPSNode : gPSNodes.values()) {
                if (gPSNode.getGPSNodeId() == 1L) {
                    continue;
                }

                // Initialize the routing architecture
                DijkstraBasedRouter dijkstraBasedRouter = new DijkstraBasedRouter();
                Node previousRoutingNode = kDTreeForDijkstra.findNearestNode(previousGPSNode.
                        getMatchedNodeLongitude(), previousGPSNode.getMatchedNodeLatitude());
                Node currentRoutingNode = kDTreeForDijkstra.findNearestNode(gPSNode.getMatchedNodeLongitude(),
                        gPSNode.getMatchedNodeLatitude());
                // System.out.println(previousGPSNode);
                // System.out.println(gPSNode.getgPSNodeTourNumber());  // Debugging statement

                // Find the inter-nodal distance between consecutive GPS nodes
                double interPointDistance = dijkstraBasedRouter.findShortestDrivingPathLengthM(
                        previousRoutingNode.getNodeId(), currentRoutingNode.getNodeId(), nodes, links);
                gPSNode.setDistanceFromPreviousGPSPointM(interPointDistance);

                // Set up an arraylist of points between consecutive GPS nodes, and populate the master list accordingly
                ArrayList<Node> nodeSequenceTraversed = dijkstraBasedRouter.createNodeListForTraversal(
                        previousRoutingNode.getNodeId(), currentRoutingNode.getNodeId(), nodes);
                // if (nodeSequenceTraversed != null) System.out.println(gPSNode.getgPSNodeTourNumber()); // Debugger

                Node intermediateNode;
                if (nodeSequenceTraversed.isEmpty()) {
                    intermediateNode = kDTreeForDijkstra.findNearestNode(gPSNode.getMatchedNodeLongitude(),
                            gPSNode.getMatchedNodeLatitude());
                    boolean isIntersection = false;

                    osmWayIdToAscribe = intermediateNode.getLinkIdList().get(0);
                    distanceFromPreviousFeatureM = intermediateNode.equiRectangularDistanceTo(previousPathNode.
                            getNodeLongitude(), previousPathNode.getNodeLatitude());


                    intermediateNode.setPreviousGPSNodeId(previousGPSNode.getGPSNodeId());
                    intermediateNode.setNextGPSNodeId(gPSNode.getGPSNodeId());
                    intermediateNode.setPreviousGPSNodeLongitude(previousGPSNode.getGPSNodeLongitude());
                    intermediateNode.setPreviousGPSNodeLatitude(previousGPSNode.getGPSNodeLatitude());
                    intermediateNode.setPreviousGPSNodeDateTime(previousGPSNode.getDateTimeStamp());
                    intermediateNode.setNextGPSNodeDateTime(gPSNode.getDateTimeStamp());
                    intermediateNode.setNextGPSNodeLongitude(gPSNode.getGPSNodeLongitude());
                    intermediateNode.setNextGPSNodeLatitude(gPSNode.getGPSNodeLatitude());
                    intermediateNode.setPreviousGPSNodeAtDepot(previousGPSNode.getgPSNodeAtDepot());
                    intermediateNode.setPreviousGPSNodeAtDepotLag(previousGPSNode.getgPSNodeAtDepotLag());
                    intermediateNode.setPreviousGPSNodeAtDepotNext(previousGPSNode.getgPSNodeAtDepotNext());
                    intermediateNode.setPreviousGPSNodeHubStay(previousGPSNode.getgPSNodeHubStay());
                    intermediateNode.setPreviousGPSNodeHubStart(previousGPSNode.getgPSNodeHubStart());
                    intermediateNode.setPreviousGPSNodeHubStop(previousGPSNode.getgPSNodeHubStop());
                    intermediateNode.setTourNumber(previousGPSNode.getgPSNodeTourNumber());
                    intermediateNode.setPreviousGPSNodeIsStop(previousGPSNode.getgPSNodeIsStop());
                    intermediateNode.setOsmWayIdAscribedForRoute(osmWayIdToAscribe);
                    intermediateNode.setDistanceToPreviousNetworkNode(distanceFromPreviousFeatureM);
                    intermediateNode.setIsIntersection(isIntersection);

                    gPSPointPathSequenceNodes.put(++pathNodeFeatureId, intermediateNode);

                    if (gPSNode.getgPSNodeTourNumber() <= 1032) {
                        pathNodeWriter.write(
                                (pathNodeFeatureId - 1) + "," +
                                        intermediateNode.getNodeId() + "," +
                                        intermediateNode.getNodeLongitude() + "," +
                                        intermediateNode.getNodeLatitude() + "," +
                                        intermediateNode.getPreviousGPSNodeId() + "," +
                                        intermediateNode.getNextGPSNodeId() + "," +
                                        intermediateNode.getPreviousGPSNodeLongitude() + "," +
                                        intermediateNode.getPreviousGPSNodeLatitude() + "," +
                                        intermediateNode.getNextGPSNodeLongitude() + "," +
                                        intermediateNode.getNextGPSNodeLatitude() + "," +
                                        intermediateNode.getPreviousGPSNodeDateTime() + "," +
                                        intermediateNode.getNextGPSNodeDateTime() + "," +
                                        intermediateNode.getPreviousGPSNodeAtDepot() + "," +
                                        intermediateNode.getPreviousGPSNodeAtDepotLag() + "," +
                                        intermediateNode.getPreviousGPSNodeAtDepotNext() + "," +
                                        intermediateNode.getPreviousGPSNodeHubStay() + "," +
                                        intermediateNode.getPreviousGPSNodeHubStart() + "," +
                                        intermediateNode.getPreviousGPSNodeHubStop() + "," +
                                        intermediateNode.getTourNumber() + "," +
                                        intermediateNode.getPreviousGPSNodeIsStop() + "," +
                                        intermediateNode.getOsmWayIdAscribedForRoute() + "," +
                                        intermediateNode.getDistanceToPreviousNetworkNode() + "," +
                                        intermediateNode.isIntersection() + "\n");
                    }
                    previousPathNode = intermediateNode;
                }

                for (int i = nodeSequenceTraversed.size() - 1; i >= 0; i--) {
                    intermediateNode = nodeSequenceTraversed.get(i);
                    boolean isIntersection = false;
                    long linkIdToCheckClass = 0L;

                    boolean breakOuterLoop = false;
                    for (Long linkIdFromPreviousNode : previousPathNode.getLinkIdList()) {
                        for (Long linkIdFromCurrentNode : intermediateNode.getLinkIdList()) {
                            if (Objects.equals(linkIdFromPreviousNode, linkIdFromCurrentNode)) {
                                osmWayIdToAscribe = links.get(linkIdFromPreviousNode).getOsmWayId();
                                distanceFromPreviousFeatureM = links.get(linkIdFromPreviousNode).
                                        getlinkLengthM();
                                linkIdToCheckClass = linkIdFromPreviousNode;
                                breakOuterLoop = true;
                            }
                        }
                        if (breakOuterLoop) break;
                    }

                    if (previousPathNode.getOsmWayIdAscribedForRoute() != osmWayIdToAscribe) {
                        if (!((Objects.equals(links.get(linkIdToCheckClass).getLinkType(), "service")) &&
                                (Objects.equals(links.get(linkIdToCheckClass).getLinkType(), "steps")) &&
                                (Objects.equals(links.get(linkIdToCheckClass).getLinkType(), "path")) &&
                                (Objects.equals(links.get(linkIdToCheckClass).getLinkType(), "footway")) &&
                                (Objects.equals(links.get(linkIdToCheckClass).getLinkType(), "cycleway")))) {
                            isIntersection = true;
                        }
                    }

                    intermediateNode.setPreviousGPSNodeId(previousGPSNode.getGPSNodeId());
                    intermediateNode.setNextGPSNodeId(gPSNode.getGPSNodeId());
                    intermediateNode.setPreviousGPSNodeLongitude(previousGPSNode.getGPSNodeLongitude());
                    intermediateNode.setPreviousGPSNodeLatitude(previousGPSNode.getGPSNodeLatitude());
                    intermediateNode.setPreviousGPSNodeDateTime(previousGPSNode.getDateTimeStamp());
                    intermediateNode.setNextGPSNodeDateTime(gPSNode.getDateTimeStamp());
                    intermediateNode.setNextGPSNodeLongitude(gPSNode.getGPSNodeLongitude());
                    intermediateNode.setNextGPSNodeLatitude(gPSNode.getGPSNodeLatitude());
                    intermediateNode.setPreviousGPSNodeAtDepot(previousGPSNode.getgPSNodeAtDepot());
                    intermediateNode.setPreviousGPSNodeAtDepotLag(previousGPSNode.getgPSNodeAtDepotLag());
                    intermediateNode.setPreviousGPSNodeAtDepotNext(previousGPSNode.getgPSNodeAtDepotNext());
                    intermediateNode.setPreviousGPSNodeHubStay(previousGPSNode.getgPSNodeHubStay());
                    intermediateNode.setPreviousGPSNodeHubStart(previousGPSNode.getgPSNodeHubStart());
                    intermediateNode.setPreviousGPSNodeHubStop(previousGPSNode.getgPSNodeHubStop());
                    intermediateNode.setTourNumber(previousGPSNode.getgPSNodeTourNumber());
                    // System.out.println(pathNodeFeatureId + "," + previousGPSNode.getgPSNodeTourNumber());
                    intermediateNode.setPreviousGPSNodeIsStop(previousGPSNode.getgPSNodeIsStop());
                    intermediateNode.setOsmWayIdAscribedForRoute(osmWayIdToAscribe);
                    intermediateNode.setDistanceToPreviousNetworkNode(distanceFromPreviousFeatureM);
                    intermediateNode.setIsIntersection(isIntersection);

                    gPSPointPathSequenceNodes.put(++pathNodeFeatureId, intermediateNode);
                        /* Debugging statement:
                        System.out.println(pathNodeFeatureId + "----" + gPSPointPathSequenceNodes.get(pathNodeFeatureId));
                        if (pathNodeFeatureId % 301_221 == 0) {
                            System.exit(007);
                        }
                        */

                    if (gPSNode.getgPSNodeTourNumber() <= 1032) {
                        pathNodeWriter.write(
                                (pathNodeFeatureId - 1) + "," +
                                        intermediateNode.getNodeId() + "," +
                                        intermediateNode.getNodeLongitude() + "," +
                                        intermediateNode.getNodeLatitude() + "," +
                                        intermediateNode.getPreviousGPSNodeId() + "," +
                                        intermediateNode.getNextGPSNodeId() + "," +
                                        intermediateNode.getPreviousGPSNodeLongitude() + "," +
                                        intermediateNode.getPreviousGPSNodeLatitude() + "," +
                                        intermediateNode.getNextGPSNodeLongitude() + "," +
                                        intermediateNode.getNextGPSNodeLatitude() + "," +
                                        intermediateNode.getPreviousGPSNodeDateTime() + "," +
                                        intermediateNode.getNextGPSNodeDateTime() + "," +
                                        intermediateNode.getPreviousGPSNodeAtDepot() + "," +
                                        intermediateNode.getPreviousGPSNodeAtDepotLag() + "," +
                                        intermediateNode.getPreviousGPSNodeAtDepotNext() + "," +
                                        intermediateNode.getPreviousGPSNodeHubStay() + "," +
                                        intermediateNode.getPreviousGPSNodeHubStart() + "," +
                                        intermediateNode.getPreviousGPSNodeHubStop() + "," +
                                        intermediateNode.getTourNumber() + "," +
                                        intermediateNode.getPreviousGPSNodeIsStop() + "," +
                                        intermediateNode.getOsmWayIdAscribedForRoute() + "," +
                                        intermediateNode.getDistanceToPreviousNetworkNode() + "," +
                                        intermediateNode.isIntersection() + "\n");
                    }
                    previousPathNode = intermediateNode;
                }
                previousGPSNode = gPSNode;
            }

            pathNodeWriter.flush();
            pathNodeWriter.close();
            System.out.println("Path nodes written to: " + pathNodesFilePath);

        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check the path nodes hashmap.");
            iOE.printStackTrace();
        }

        /* Debugging statement: Most important debugging step thus far
        for (int i = 1; i < gPSPointPathSequenceNodes.size(); i++) {
            System.out.println(i + "----" + gPSPointPathSequenceNodes.get(i));
            if (i % 5000 == 0) {
                System.exit(007);
            }
        }
        */

        System.out.println(gPSPointPathSequenceNodes.size());
        writeSnappedGPSNodes(snappedGPSNodesFilePath, gPSNodes);
        developOptimalPathNodeSequence(gPSNodes, gPSPointIsStopPathSequenceNodes, links, nodes, kDTreeForDijkstra);
        writeStopPointShortestPathNodes(gPSPointIsStopPathSequenceNodes, shortestPathNodesFilePath);
    }

    // Calculate emission probabilities for a GPS node and candidate state (network) nodes
    private static ArrayList<Double> calculateEmissionProbabilities(GPSNode gPSNode) {
        ArrayList<NetworkNode> candidateStateNodes = gPSNode.getCandidateStateNodes();
        ArrayList<Double> emissionProbabilities = new ArrayList<>();
        int scalingParameter = 70;  // Distance beyond which sharp decay occurs

        for (NetworkNode candidateStateNode : candidateStateNodes) {
            double interPointDistance = calculateEuclideanDistance(gPSNode, candidateStateNode);
            double emissionProbability = Math.exp(-1 * interPointDistance / scalingParameter);
            emissionProbabilities.add(emissionProbability);
        }

        return emissionProbabilities;
    }

    // Calculate transition probabilities between two state nodes using internode distance and connectivity matrix
    private static ArrayList<Double> calculateTransitionProbabilities(NetworkNode fromCandidateState,
                                                                      ArrayList<NetworkNode> toCandidateStates,
                                                                      LinkedHashMap<Long, ArrayList<Long>>
                                                                              connectivityMatrix) {
        ArrayList<Double> transitionProbabilities = new ArrayList<>();
        int scalingParameter = 70;

        for (NetworkNode toCandidateState : toCandidateStates) {
            double interPointDistance = calculateEuclideanDistance(fromCandidateState, toCandidateState);
            double transitionProbability = 0;

            // Check for null values
            if (connectivityMatrix != null && connectivityMatrix.get(fromCandidateState.getRoadOsmId()) != null &&
                    toCandidateState.getRoadOsmId() != 0) {
                // Compute transition probability
                transitionProbability = connectivityMatrix.get(fromCandidateState.getRoadOsmId()).
                        contains(toCandidateState.getRoadOsmId()) ? Math.exp(-1 * interPointDistance /
                        scalingParameter) : 0;
            }
            transitionProbabilities.add(transitionProbability);
        }

        return transitionProbabilities;
    }

    // Determine the distance between a GPS node and a network node
    private static double calculateEuclideanDistance(GPSNode gPSNode, NetworkNode networkNode) {
        double interPointDistance;
        int radiusOfEarthM = 6_371_000;

        double xDifferential = (networkNode.getNetworkNodeLongitude() - gPSNode.getGPSNodeLongitude()) *
                Math.cos(gPSNode.getGPSNodeLatitude() * Math.PI / 180) * radiusOfEarthM;
        double yDifferential = (networkNode.getNetworkNodeLatitude() - gPSNode.getGPSNodeLatitude()) *
                radiusOfEarthM;
        interPointDistance = Math.sqrt(xDifferential * xDifferential + yDifferential * yDifferential);

        return interPointDistance;
    }

    // Determine the distance between two network nodes
    private static double calculateEuclideanDistance(NetworkNode networkNode1,
                                                     NetworkNode networkNode2) {
        double interPointDistance = 3;
        int radiusOfEarthM = 6_371_000;

        double xDifferential = (networkNode1.getNetworkNodeLongitude() - networkNode2.getNetworkNodeLongitude()) *
                Math.cos(networkNode2.getNetworkNodeLatitude() * Math.PI / 180) * radiusOfEarthM;
        double yDifferential = (networkNode1.getNetworkNodeLatitude() - networkNode2.getNetworkNodeLatitude()) *
                radiusOfEarthM;
        interPointDistance = Math.sqrt(xDifferential * xDifferential + yDifferential * yDifferential);

        return interPointDistance;
    }

    // Write out the snapped GPS nodes
    private static void writeSnappedGPSNodes(String gPSOutputFilePath, LinkedHashMap<Long, GPSNode> gPSNodes) {
        try {
            // Set up a writer instance
            BufferedWriter gPSNodeWriter = new BufferedWriter(new FileWriter(gPSOutputFilePath));

            // Write out the header
            gPSNodeWriter.write("GPSNodeId,GPSNodeLongitude,GPSNodeLatitude,DateTimeStamp," +
                    "MatchedNodeLongitude,MatchedNodeLatitude,MatchedNodeElevationM,MatchedNodeOsmWayId," +
                    "PathDistanceToPreviousPointM,TourNumber,JointProbability" +
                    "\n");

            // Write out the data
            for (GPSNode gPSNode : gPSNodes.values()) {
                gPSNodeWriter.write(gPSNode.getGPSNodeId() + "," +
                        gPSNode.getGPSNodeLongitude() + "," +
                        gPSNode.getGPSNodeLatitude() + "," +
                        gPSNode.getDateTimeStamp() + "," +
                        gPSNode.getMatchedNodeLongitude() + "," +
                        gPSNode.getMatchedNodeLatitude() + "," +
                        gPSNode.getMatchedNodeElevationM() + "," +
                        gPSNode.getMatchedNodeOsmWayId() + "," +
                        gPSNode.getDistanceFromPreviousPointM() + "," +
                        gPSNode.getgPSNodeTourNumber() + "," +
                        gPSNode.getJointProbability() +
                        "\n");
            }
            gPSNodeWriter.flush();
            gPSNodeWriter.close();
            System.out.println("Snapped GPS nodes written to: " + gPSOutputFilePath);

        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check the GPS nodes hashmap.");
        }
    }

    // Handle the development of optimal path sequence nodes
    private static void developOptimalPathNodeSequence(LinkedHashMap<Long, GPSNode> gPSNodes,
                                                       LinkedHashMap<Integer, Node> stopPointShortestPathNodes,
                                                       LinkedHashMap<Long, Link> links,
                                                       LinkedHashMap<Long, Node> nodes,
                                                       KDTreeForNodes kDTreeForNodes) {

        GPSNode previousIsStopNode = null;
        int nodeSequenceCounter = 0;
        Node previousOptimalPathNode = null;
        long osmWayIdToAscribe = 0;
        double distanceToPreviousFeature = 0;

        for (GPSNode gPSNode : gPSNodes.values()) {
            if (gPSNode.getgPSNodeIsStop().equalsIgnoreCase("TRUE")) {
                previousIsStopNode = gPSNode;
                break;
            }
        }

        for (GPSNode gPSNode : gPSNodes.values()) {
            if (((gPSNode.getgPSNodeIsStop().equalsIgnoreCase("TRUE")) || (gPSNode.getgPSNodeHubStart().
                    equalsIgnoreCase("TRUE"))) && (gPSNode != previousIsStopNode)) {
                DijkstraBasedRouter dijkstraBasedRouter = new DijkstraBasedRouter();
                assert previousIsStopNode != null;
                Node originNode = kDTreeForNodes.findNearestNode(previousIsStopNode.getMatchedNodeLongitude(),
                        previousIsStopNode.getMatchedNodeLatitude());
                Node destinationNode = kDTreeForNodes.findNearestNode(gPSNode.getMatchedNodeLongitude(),
                        gPSNode.getMatchedNodeLatitude());
                double lengthOfShortestPath = dijkstraBasedRouter.findShortestDrivingPathLengthM(originNode.getNodeId(),
                        destinationNode.getNodeId(), nodes, links);
                ArrayList<Node> nodeListForTraversal = dijkstraBasedRouter.createNodeListForTraversal(
                        originNode.getNodeId(), destinationNode.getNodeId(), nodes);

                if (nodeListForTraversal.isEmpty()) {
                    nodeListForTraversal.add(kDTreeForNodes.findNearestNode(previousIsStopNode.getGPSNodeLongitude(),
                            previousIsStopNode.getGPSNodeLatitude()));
                }

                for (int i = nodeListForTraversal.size() - 1; i >= 0; i--) {
                    Node nodeInOptimalPath = nodeListForTraversal.get(i);
                    nodeInOptimalPath.setPreviousGPSNodeId(previousIsStopNode.getGPSNodeId());
                    nodeInOptimalPath.setNextGPSNodeId(gPSNode.getGPSNodeId());
                    nodeInOptimalPath.setPreviousGPSNodeLongitude(previousIsStopNode.getGPSNodeLongitude());
                    nodeInOptimalPath.setPreviousGPSNodeLatitude(previousIsStopNode.getGPSNodeLatitude());
                    nodeInOptimalPath.setPreviousGPSNodeDateTime(previousIsStopNode.getDateTimeStamp());
                    nodeInOptimalPath.setNextGPSNodeDateTime(gPSNode.getDateTimeStamp());
                    nodeInOptimalPath.setNextGPSNodeLongitude(gPSNode.getGPSNodeLongitude());
                    nodeInOptimalPath.setNextGPSNodeLatitude(gPSNode.getGPSNodeLatitude());
                    nodeInOptimalPath.setPreviousGPSNodeAtDepot(previousIsStopNode.getgPSNodeAtDepot());
                    nodeInOptimalPath.setPreviousGPSNodeAtDepotLag(previousIsStopNode.getgPSNodeAtDepotLag());
                    nodeInOptimalPath.setPreviousGPSNodeAtDepotNext(previousIsStopNode.getgPSNodeAtDepotNext());
                    nodeInOptimalPath.setPreviousGPSNodeHubStay(previousIsStopNode.getgPSNodeHubStay());
                    nodeInOptimalPath.setPreviousGPSNodeHubStart(previousIsStopNode.getgPSNodeHubStart());
                    nodeInOptimalPath.setPreviousGPSNodeHubStop(previousIsStopNode.getgPSNodeHubStop());
                    nodeInOptimalPath.setTourNumber(previousIsStopNode.getgPSNodeTourNumber());
                    nodeInOptimalPath.setPreviousGPSNodeIsStop(previousIsStopNode.getgPSNodeIsStop());

                    long linkIdForIntersection = 0;
                    if (previousOptimalPathNode != null) {
                        for (Long osmLinkIdFromPreviousOptimalNode : previousOptimalPathNode.getLinkIdList()) {
                            for (Long osmLinkIdFromCurrentOptimalNode : nodeInOptimalPath.getLinkIdList()) {
                                if (Objects.equals(osmLinkIdFromPreviousOptimalNode, osmLinkIdFromCurrentOptimalNode)) {
                                    linkIdForIntersection = osmLinkIdFromCurrentOptimalNode;
                                    osmWayIdToAscribe = links.get(osmLinkIdFromCurrentOptimalNode).getOsmWayId();
                                    distanceToPreviousFeature = links.get(osmLinkIdFromCurrentOptimalNode).
                                            getlinkLengthM();
                                }
                            }
                        }
                    }
                    nodeInOptimalPath.setDistanceToPreviousNetworkNode(distanceToPreviousFeature);
                    nodeInOptimalPath.setOsmWayIdAscribedForRoute(osmWayIdToAscribe);

                    boolean isIntersection = false;
                    if (previousOptimalPathNode != null) {
                        if (!Objects.equals(previousOptimalPathNode.getOsmWayIdAscribedForRoute(), osmWayIdToAscribe)) {
                            nodeInOptimalPath.setIsIntersection(!links.get(linkIdForIntersection).getLinkType().
                                    equalsIgnoreCase("service"));
                        } else {
                            nodeInOptimalPath.setIsIntersection(false);
                        }
                    }

                    stopPointShortestPathNodes.put(++nodeSequenceCounter, nodeInOptimalPath);
                    previousOptimalPathNode = nodeInOptimalPath;
                }

                previousIsStopNode = gPSNode;
            }
        }
        System.out.println("Optimal paths' node sequence established.");
    }

    // Write out the optimal sequence of nodes based on the list of acquired stop points
    public static void writeStopPointShortestPathNodes(LinkedHashMap<Integer, Node> stopPointSequence,
                                                       String stopPointOptimalSequenceFilePath) {
        System.out.println(stopPointSequence.size());
        try {
            // initialize a writer
            BufferedWriter optimalPathNodeSequenceWriter = new BufferedWriter(new FileWriter(
                    stopPointOptimalSequenceFilePath));

            // Write the header array
            optimalPathNodeSequenceWriter.write("FeatureId,OSMNodeId,FeatureLongitude,FeatureLatitude," +
                    "FromGPSPointId,ToGPSPointId,FromGPSPointLongitude,FromGPSPointLatitude,ToGPSPointLongitude," +
                    "ToGPSPointLatitude,FromGPSPointDateTime,ToGPSPointDateTime,FromGPSPointAtDepot," +
                    "FromGPSPointAtDepotLag,FromGPSPointAtDepotNext,FromGPSPointHubStay,FromGPSPointHubStart," +
                    "FromGPSPointHubStop,FromGPSPointTourNumber,FromGPSPointIsStop,OSMWayId," +
                    "DistanceFromPreviousFeature,IsIntersection\n");

            // Write out the data body
            for (HashMap.Entry<Integer, Node> optimalPathNodeEntry : stopPointSequence.entrySet()) {
                int pathSequenceNodeId = optimalPathNodeEntry.getKey();
                Node pathSequenceNode = optimalPathNodeEntry.getValue();
                optimalPathNodeSequenceWriter.write(pathSequenceNodeId + "," +
                        pathSequenceNode.getNodeId() + "," +
                        pathSequenceNode.getNodeLongitude() + "," +
                        pathSequenceNode.getNodeLatitude() + "," +
                        pathSequenceNode.getPreviousGPSNodeId() + "," +
                        pathSequenceNode.getNextGPSNodeId() + "," +
                        pathSequenceNode.getPreviousGPSNodeLongitude() + "," +
                        pathSequenceNode.getPreviousGPSNodeLatitude() + "," +
                        pathSequenceNode.getNextGPSNodeLongitude() + "," +
                        pathSequenceNode.getNextGPSNodeLatitude() + "," +
                        pathSequenceNode.getPreviousGPSNodeDateTime() + "," +
                        pathSequenceNode.getNextGPSNodeDateTime() + "," +
                        pathSequenceNode.getPreviousGPSNodeAtDepot() + "," +
                        pathSequenceNode.getPreviousGPSNodeAtDepotLag() + "," +
                        pathSequenceNode.getPreviousGPSNodeAtDepotNext() + "," +
                        pathSequenceNode.getPreviousGPSNodeHubStay() + "," +
                        pathSequenceNode.getPreviousGPSNodeHubStart() + "," +
                        pathSequenceNode.getPreviousGPSNodeHubStop() + "," +
                        pathSequenceNode.getTourNumber() + "," +
                        pathSequenceNode.getPreviousGPSNodeIsStop() + "," +
                        pathSequenceNode.getOsmWayIdAscribedForRoute() + "," +
                        pathSequenceNode.getDistanceToPreviousNetworkNode() + "," +
                        pathSequenceNode.isIntersection() +
                        "\n");
            }

            // Close writing activity
            optimalPathNodeSequenceWriter.flush();
            optimalPathNodeSequenceWriter.close();
            System.out.println("Node sequence of optimal paths between stop points written out to " +
                    stopPointOptimalSequenceFilePath);


        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check the optimal paths' node sequence hashmap.");
            iOE.printStackTrace();
        }
    }
}