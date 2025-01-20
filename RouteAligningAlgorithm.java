/**
 * Authors: Anirudh Ray and Matthias Länger
 * Institution: Professorship of Travel Behaviour, Technical University of Munich
 * Department: Mobility Systems Engineering, School of Engineering and Design
 * E-mail Address: Anirudh.Ray@tum.de
 * Purpose: Component of a Java-based map-matching algorithm and GPS data analyser
 */

import DijkstraAlgorithm.*;
import GPSDataManager.*;
import KDTreeManager.KDTreeBuilderSearcher;
import OSMDataManager.*;

import java.io.*;
import java.util.*;

public class RouteAligningAlgorithm {
    public static void main(String[] args) {
        String gPSNodesFilePath = "E:/anirudh/CargoBikeProject/RawData/GPSData/2024-09-04_GPSrecords_edited.csv";
        String hMMNodesFilePath = "E:/anirudh/CargoBikeProject/RawData/Osmium/" +
                "OSMRoadNetworkForCargoBikes1mPointsHMM.csv";
        String dijkstraLinksFilePath = "E:/anirudh/CargoBikeProject/Results/dijkstraLinks.csv";
        String dijkstraNodesFilePath = "E:/anirudh/CargoBikeProject/Results/dijkstraNodes.csv";
        int numberNeighbouringNetworkNodesCompared = 70;
        String routingNetworkFilePath = "E:/anirudh/CargoBikeProject/RawData/Osmium/" +
                "planet_11.229,48_11.914,48.278.osm.opl";
        String snappedGPSNodesFilePath = "E:/anirudh/CargoBikeProject/Results/gPSNodesSnapped.csv";
        String pathNodesFilePath = "E:/anirudh/CargoBikeProject/Results/pathNodes.csv";

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
        // osmDataReaderWriter.writeDijkstraLinks(dijkstraLinksFilePath);
        // osmDataReaderWriter.writeDijkstraNodes(dijkstraNodesFilePath);
        LinkedHashMap<Integer, Node> pathSequenceNodes = new LinkedHashMap<>();

        Node[] nodesForSnapping = nodes.values().toArray(new Node[0]);
        KDTreeForNodes kdTreeForNodes = new KDTreeForNodes();
        kdTreeForNodes.buildNodeBasedKDTree(nodesForSnapping);

        // Find the candidate network nodes for each GPS node, i.e., carry out map matching
        KDTreeBuilderSearcher kdTreeBuilderSearcher = new KDTreeBuilderSearcher();
        NetworkNode[] networkNodesForKDTree = networkNodes.values().toArray(new NetworkNode[0]);
        kdTreeBuilderSearcher.buildNodeBasedKDTree(networkNodesForKDTree);

        NetworkNode previousState = null;
        long jointProbabilityCounter = 0;
        for (GPSNode gPSNode : gPSNodes.values()) {
            // System.out.println("GPS node ID: " + gPSNode.getGPSNodeId());
            List<NetworkNode> candidateStateNodes = kdTreeBuilderSearcher.findNearestNodes(gPSNode.
                    getGPSNodeLongitude(), gPSNode.getGPSNodeLatitude(), numberNeighbouringNetworkNodesCompared);
            gPSNode.getCandidateStateNodes().addAll(candidateStateNodes);
            gPSNode.setEmissionProbabilities(calculateEmissionProbabilities(gPSNode));
            ArrayList<Double> emissionProbabilities = gPSNode.getEmissionProbabilities();
            double maximumJointProbability = -1;    // Set to -1 from 0; algorithmic intervention
            int indexOfSelectedState = 0;

            if (previousState != null) {
                ArrayList<Double> transitionProbabilities = calculateTransitionProbabilities(previousState,
                        gPSNode.getCandidateStateNodes(), connectivityMatrix);
                for (int i = 0; i < candidateStateNodes.size(); i++) {
                    double jointProbability = emissionProbabilities.get(i) * transitionProbabilities.get(i);
                    /* Debugging statements:
                    jointProbabilityCounter++;
                    System.out.println(jointProbability);
                    */
                    if (jointProbability >= maximumJointProbability) {
                        maximumJointProbability = jointProbability;
                        indexOfSelectedState = i;
                    }
                }
            } else {
                for (int i = 0; i < candidateStateNodes.size(); i++) {
                    double startingProbability = emissionProbabilities.get(i);
                    // System.out.println(startingProbability);
                    if (startingProbability >= maximumJointProbability) {
                        maximumJointProbability = startingProbability;
                        indexOfSelectedState = i;
                    }
                }
            }

            previousState = candidateStateNodes.get(indexOfSelectedState);
            gPSNode.setMatchedNodeLongitude(previousState.getNetworkNodeLongitude());
            gPSNode.setMatchedNodeLatitude(previousState.getNetworkNodeLatitude());
            gPSNode.setMatchedNodeOsmWayId(previousState.getRoadOsmId());
            gPSNode.setJointProbability(maximumJointProbability);
            gPSNode.setMatchedNodeElevationM(0);
        }

        try {
            // Set up a writer instance
            BufferedWriter pathNodeWriter = new BufferedWriter(new FileWriter(pathNodesFilePath));

            // Write out the header
            pathNodeWriter.write("FeatureId,OSMNodeId,FeatureLongitude,FeatureLatitude,FromGPSPointId," +
                    "ToGPSPointId,FromGPSPointLongitude,FromGPSPointLatitude,ToGPSPointLongitude,ToGPSPointLatitude," +
                    "FromGPSPointDateTime,ToGPSPointDateTime,FromGPSPointAtDepot,FromGPSPointAtDepotLag," +
                    "FromGPSPointAtDepotNext,FromGPSPointHubStay,FromGPSPointHubStart,FromGPSPointHubStop," +
                    "FromGPSPointTourNumber,FromGPSPointIsStop,OSMWayId,DistanceFromPreviousFeature,IsIntersection," +
                    "\n");

            // Route between different network nodes (states) and feed the arraylists to the master map
            int nodeSequenceCounter = 1;
            gPSNodes.get(1L).setDistanceFromPreviousPointM(0);  // Set starting point's distance to its predecessor as zero
            pathSequenceNodes.put(1, kdTreeForNodes.findNearestNode(gPSNodes.get(1L).getMatchedNodeLongitude(),
                    gPSNodes.get(1L).getMatchedNodeLatitude()));    // Initial value handling
            GPSNode previousNode = null;
            Node previousNetworkNode = null;
            long osmWayIdToAscribe = 0L;
            double distanceFromPreviousFeatureM = 0;

            for (GPSNode gPSNode : gPSNodes.values()) {
                // System.out.println(gPSNode.getgPSNodeTourNumber());  // Debugging statement
                if (previousNode != null) {
                    // Initialize a routing engine
                    DijkstraBasedRouter dijkstraBasedRouter = new DijkstraBasedRouter();

                    // Ready the nodes to be routed to and from
                    Node previousRoutingNode = kdTreeForNodes.findNearestNode(previousNode.getMatchedNodeLongitude(),
                            previousNode.getMatchedNodeLatitude());
                    Node currentRoutingNode = kdTreeForNodes.findNearestNode(gPSNode.getMatchedNodeLongitude(),
                            gPSNode.getMatchedNodeLatitude());
                    // System.out.println(previousNode);

                    // Find the inter-nodal distance between consecutive GPS nodes
                    double interPointDistance = dijkstraBasedRouter.findShortestDrivingPathLengthM(
                            previousRoutingNode.getNodeId(), currentRoutingNode.getNodeId(), nodes, links);
                    gPSNode.setDistanceFromPreviousPointM(interPointDistance);

                    // Set up an arraylist of points between consecutive GPS nodes, and populate the master list accordingly
                    ArrayList<Node> nodeSequenceTraversed = dijkstraBasedRouter.createNodeListForTraversal(
                            previousRoutingNode.getNodeId(), currentRoutingNode.getNodeId(), nodes);
                    // if (nodeSequenceTraversed != null) System.out.println(gPSNode.getgPSNodeTourNumber()); // Debugger

                    // TODO IF NODESEQUENCE TRAVSERSED IS NULL

                    Node intermediateNode;

                    if (nodeSequenceTraversed.isEmpty()) {
                        intermediateNode = kdTreeForNodes.findNearestNode(gPSNode.getMatchedNodeLongitude(),
                                gPSNode.getMatchedNodeLatitude());
                        boolean isIntersection = false;
                        long linkIdToCheckClass = 0;

                        if (previousNetworkNode != null) {
                            osmWayIdToAscribe = intermediateNode.getLinkIdList().get(0);
                            distanceFromPreviousFeatureM = intermediateNode.equiRectangularDistanceTo(previousNetworkNode.
                                    getNodeLongitude(), previousNetworkNode.getNodeLatitude());
                        }

                        intermediateNode.setPreviousGPSNodeId(previousNode.getGPSNodeId());
                        intermediateNode.setNextGPSNodeId(gPSNode.getGPSNodeId());
                        intermediateNode.setPreviousGPSNodeLongitude(previousNode.getGPSNodeLongitude());
                        intermediateNode.setPreviousGPSNodeLatitude(previousNode.getGPSNodeLatitude());
                        intermediateNode.setPreviousGPSNodeDateTime(previousNode.getDateTimeStamp());
                        intermediateNode.setNextGPSNodeDateTime(gPSNode.getDateTimeStamp());
                        intermediateNode.setNextGPSNodeLongitude(gPSNode.getGPSNodeLongitude());
                        intermediateNode.setNextGPSNodeLatitude(gPSNode.getGPSNodeLatitude());
                        intermediateNode.setPreviousGPSNodeAtDepot(previousNode.getgPSNodeAtDepot());
                        intermediateNode.setPreviousGPSNodeAtDepotLag(previousNode.getgPSNodeAtDepotLag());
                        intermediateNode.setPreviousGPSNodeAtDepotNext(previousNode.getgPSNodeAtDepotNext());
                        intermediateNode.setPreviousGPSNodeHubStay(previousNode.getgPSNodeHubStay());
                        intermediateNode.setPreviousGPSNodeHubStart(previousNode.getgPSNodeHubStart());
                        intermediateNode.setPreviousGPSNodeHubStop(previousNode.getgPSNodeHubStop());
                        intermediateNode.setTourNumber(previousNode.getgPSNodeTourNumber());
                        // System.out.println(nodeSequenceCounter + "," + previousNode.getgPSNodeTourNumber());
                        intermediateNode.setPreviousGPSNodeIsStop(previousNode.getgPSNodeIsStop());
                        intermediateNode.setOsmWayIdAscribedForRoute(osmWayIdToAscribe);
                        intermediateNode.setDistanceToPreviousNetworkNode(distanceFromPreviousFeatureM);
                        intermediateNode.setIsIntersection(isIntersection);

                        pathSequenceNodes.put(++nodeSequenceCounter, intermediateNode);

                        if (gPSNode.getgPSNodeTourNumber() <= 1032) {   // todo earlier is was nodeSequenceCounter % 301_122
                            pathNodeWriter.write(
                                    (nodeSequenceCounter - 1) + "," +
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
                        previousNetworkNode = intermediateNode;
                    }

                    for (int i = nodeSequenceTraversed.size() - 1; i >= 0; i--) {
                        intermediateNode = nodeSequenceTraversed.get(i);
                        boolean isIntersection = false;
                        long linkIdToCheckClass = 0L;

                        if (previousNetworkNode != null) {
                            boolean breakOuterLoop = false;
                            for (Long linkIdFromPreviousNode : previousNetworkNode.getLinkIdList()) {
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

                            if (previousNetworkNode.getOsmWayIdAscribedForRoute() != osmWayIdToAscribe) {
                                if (!Objects.equals(links.get(linkIdToCheckClass).getLinkType(), "service")) {
                                    isIntersection = true;
                                }
                            }
                        }

                        intermediateNode.setPreviousGPSNodeId(previousNode.getGPSNodeId());
                        intermediateNode.setNextGPSNodeId(gPSNode.getGPSNodeId());
                        intermediateNode.setPreviousGPSNodeLongitude(previousNode.getGPSNodeLongitude());
                        intermediateNode.setPreviousGPSNodeLatitude(previousNode.getGPSNodeLatitude());
                        intermediateNode.setPreviousGPSNodeDateTime(previousNode.getDateTimeStamp());
                        intermediateNode.setNextGPSNodeDateTime(gPSNode.getDateTimeStamp());
                        intermediateNode.setNextGPSNodeLongitude(gPSNode.getGPSNodeLongitude());
                        intermediateNode.setNextGPSNodeLatitude(gPSNode.getGPSNodeLatitude());
                        intermediateNode.setPreviousGPSNodeAtDepot(previousNode.getgPSNodeAtDepot());
                        intermediateNode.setPreviousGPSNodeAtDepotLag(previousNode.getgPSNodeAtDepotLag());
                        intermediateNode.setPreviousGPSNodeAtDepotNext(previousNode.getgPSNodeAtDepotNext());
                        intermediateNode.setPreviousGPSNodeHubStay(previousNode.getgPSNodeHubStay());
                        intermediateNode.setPreviousGPSNodeHubStart(previousNode.getgPSNodeHubStart());
                        intermediateNode.setPreviousGPSNodeHubStop(previousNode.getgPSNodeHubStop());
                        intermediateNode.setTourNumber(previousNode.getgPSNodeTourNumber());
                        // System.out.println(nodeSequenceCounter + "," + previousNode.getgPSNodeTourNumber());
                        intermediateNode.setPreviousGPSNodeIsStop(previousNode.getgPSNodeIsStop());
                        intermediateNode.setOsmWayIdAscribedForRoute(osmWayIdToAscribe);
                        intermediateNode.setDistanceToPreviousNetworkNode(distanceFromPreviousFeatureM);
                        intermediateNode.setIsIntersection(isIntersection);

                        pathSequenceNodes.put(++nodeSequenceCounter, intermediateNode);
                        /* Debugging statement:
                        System.out.println(nodeSequenceCounter + "----" + pathSequenceNodes.get(nodeSequenceCounter));
                        if (nodeSequenceCounter % 301_221 == 0) {
                            System.exit(007);
                        }
                        */

                        if (gPSNode.getgPSNodeTourNumber() <= 1032) {   // todo earlier is was nodeSequenceCounter % 301_122
                            pathNodeWriter.write(
                                    (nodeSequenceCounter - 1) + "," +
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
                        previousNetworkNode = intermediateNode;
                    }
                }
                previousNode = gPSNode;
            }

            pathNodeWriter.flush();
            pathNodeWriter.close();
            System.out.println("Path nodes written to: " + pathNodesFilePath);

        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check the path nodes hashmap.");
            iOE.printStackTrace();
        }

        /* Debugging statements:
        for (Node intermediateNode : pathSequenceNodes.values()) {
            System.out.println("Intermediate node details: " +
                    intermediateNode.getPreviousGPSNodeId() + "," +
                    intermediateNode.getPreviousGPSNodeId() + "," +
                    intermediateNode.getPreviousGPSNodeLongitude() + "," +
                    intermediateNode.getPreviousGPSNodeLatitude() + "," +
                    intermediateNode.getNextGPSNodeLongitude() + "," +
                    intermediateNode.getNextGPSNodeLatitude() + "," +
                    intermediateNode.getPreviousGPSNodeAtDepot() + "," +
                    intermediateNode.getPreviousGPSNodeAtDepotLag() + "," +
                    intermediateNode.getPreviousGPSNodeAtDepotNext() + "," +
                    intermediateNode.getPreviousGPSNodeHubStay() + "," +
                    intermediateNode.getPreviousGPSNodeHubStart() + "," +
                    intermediateNode.getPreviousGPSNodeHubStop() + "," +
                    intermediateNode.getTourNumber() + "," +
                    intermediateNode.getPreviousGPSNodeIsStop() + "," +
                    intermediateNode.getOsmWayIdAscribedForRoute() + "," +
                    intermediateNode.getDistanceToPreviousNetworkNode());

            if (nodeSequenceCounter % 500 == 0) {
                System.exit(23);
            }
        }
        */

        /* Debugging statement: TODO: Most important debugging step thus far
        for (int i = 1; i < pathSequenceNodes.size(); i++) {
            System.out.println(i + "----" + pathSequenceNodes.get(i));
            if (i % 5000 == 0) {
                System.exit(007);
            }
        }
        */

            writeSnappedGPSNodes(snappedGPSNodesFilePath, gPSNodes);
            writePathNodes(pathNodesFilePath, pathSequenceNodes);
        }

    // Calculate emission probabilities for a GPS node and candidate state (network) nodes
    private static ArrayList<Double> calculateEmissionProbabilities(GPSNode gPSNode) {
        ArrayList<NetworkNode> candidateStateNodes = gPSNode.getCandidateStateNodes();
        ArrayList<Double> emissionProbabilities = new ArrayList<>();
        int scalingParameter = 70;

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
                                                                              connectivityMatrix)
    {
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
        double interPointDistance = 7;
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

    // Write out the pathfinding nodes
    private static void writePathNodes(String pathNodesOutputFilePath, LinkedHashMap<Integer, Node> nodesForSequencing)
    {

    }
}
