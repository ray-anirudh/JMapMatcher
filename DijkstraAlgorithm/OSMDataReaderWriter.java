/**
 * Author: Anirudh Ray
 * Institution: Professorship of Traffic Engineering and Control, Technical University of Munich
 * Department: Mobility Systems Engineering, School of Engineering and Design
 * E-mail Address: Anirudh.Ray@tum.de
 * Purpose: Component of a Java-based multi-modal routing algorithm, built using RAPTOR, Dijkstra-algorithm, and
 * KD-Trees
 */

package DijkstraAlgorithm;
// OSM: OpenStreetMap
// OPL: Object-Per-Line (format)

import java.io.*;
import java.util.*;

public class OSMDataReaderWriter {

    /**
     * ATTRIBUTE DEFINITIONS
     */

    /* Array to limit the way types (classes) parsed out of the OSM-OPL extract
    Refer to: https://wiki.openstreetmap.org/wiki/Key:highway for more details
    */
    private static final String[] LINK_TYPE_ARRAY = {
            // "bridleway",
            // "cycleway",
            // "footway",
            // "living_street",
            // "motorway",
            // "motorway_link",
            // "path",
            "pedestrian",
            "primary",
            "primary_link",
            "residential",
            "secondary",
            "secondary_link",
            "service",
            // "steps",
            "tertiary",
            "tertiary_link",
            // "track",
            // "track_grade1",
            // "track_grade2",
            // "track_grade3",
            // "track_grade4",
            // "track_grade5",
            // "trunk",
            // "trunk_link",
            // "unclassified"
    };

    // Initialize the Dijkstra-relevant hashmaps
    // Keys in "nodes" hashmap refer to node IDs, and values pertain to nodal coordinates and lists of associated links
    LinkedHashMap<Long, Node> nodes = new LinkedHashMap<>();

    // Keys in "links" hashmap refer to link IDs, and values pertain to associated nodes, link types, and travel times
    LinkedHashMap<Long, Link> links = new LinkedHashMap<>();

    /**
     * BEHAVIOUR DEFINITIONS
     * All readers and dataset manipulators below are for Dijkstra-relevant data, and sourced from OSM files
     */

    // Build "links" hashmap from the OSM-OPL extract
    public void readAndFilterOsmLinks(String osmOplExtractFilePath) {
        try {
            // Reader for first record of "BBBikeOSMExtract.opl"
            BufferedReader osmOplExtractReader = new BufferedReader(new FileReader(osmOplExtractFilePath));
            String newline;

            // Find relevant indices using the first OPL data record
            String[] firstOsmOplRecordArray = osmOplExtractReader.readLine().split(" ");
            // First object is a node
            int linkAttributesIndex = findIndexInArray("T", firstOsmOplRecordArray);
            int osmWayIdIndex = 0;
            int linkNodesIndex = linkAttributesIndex + 1;

            // Create link types' hashset for filtering way records pertaining to the road network
            HashSet<String> linkTypeHashSet = new HashSet<>(Arrays.asList(LINK_TYPE_ARRAY));

            // Read body and process data for all links in the road network
            long wayId = 0;
            while ((newline = osmOplExtractReader.readLine()) != null) {
                wayId++;
                if (newline.substring(0, 1).equalsIgnoreCase("w")) {
                    String[] linkDataRecord = newline.split(" ");
                    long osmWayId = Long.parseLong(linkDataRecord[osmWayIdIndex].substring(1));
                    // System.out.println("OSM Way ID: " + osmWayId);   // Debugging statement
                    String linkAttributes = linkDataRecord[linkAttributesIndex];

                    if (linkAttributes.contains("highway")) {
                        String[] linkAttributesRecord = linkAttributes.split(",");
                        int linkTypeIndex = findIndexInArray("highway", linkAttributesRecord);
                        /* Example of a link's record of details: Thighway=track,maxspeed:type=DE:rural,surface=asphalt,
                        tracktype=grade1 Nn1755165066,n1755165067,n262608882
                        */

                        String linkType = linkAttributesRecord[linkTypeIndex].substring((linkTypeIndex == 0) ? 9 : 8);
                        if (linkTypeHashSet.contains(linkType)) {
                            String[] nodesArray = linkDataRecord[linkNodesIndex].split(",");

                            for (int i = 0; i <= nodesArray.length - 2; i++) {
                                // "first" and "second" nodes do not signify any ordering of nodes/ links
                                long firstNodeId;
                                firstNodeId = Long.parseLong(nodesArray[i].substring((i == 0) ? 2 : 1));
                                long secondNodeId;
                                secondNodeId = Long.parseLong(nodesArray[i + 1].substring(1));
                                long linkId = Long.parseLong(wayId + "0" + (i + 1) + "00");
                                Link link = new Link(firstNodeId, secondNodeId, linkType, osmWayId);

                                this.links.put(linkId, link);
                                ArrayList<Long> firstNodeLinkIdList = new ArrayList<>();
                                ArrayList<Long> secondNodeLinkIdList = new ArrayList<>();
                                this.nodes.put(firstNodeId, new Node(firstNodeLinkIdList));
                                this.nodes.put(secondNodeId, new Node(secondNodeLinkIdList));
                            }
                        }
                    }
                }
            }
            System.out.println("Links' data read from " + osmOplExtractFilePath);

        } catch (FileNotFoundException fNFE) {
            System.out.println("File not found at specified path: " + osmOplExtractFilePath);
        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check input file: " + osmOplExtractFilePath);
        }
    }

    // Remove links starting and ending at the same node (helpful against exceptional cases)
    public void removeCircularLinks() {
        ArrayList<Long> linkIdsList = new ArrayList<>(this.links.keySet());
        for (long linkId : linkIdsList) {
            if (this.links.get(linkId).getFirstNodeId() == this.links.get(linkId).getSecondNodeId()) {
                this.links.remove(linkId);
            }
        }
        System.out.println("Circular links removed");
    }

    // Build "nodes" hashmap from the OSM-OPL extract
    public void readAndFilterOsmNodes(String osmOplExtractFilePath) {
        // Initializing required indices
        int nodeIdIndex = 0;
        int longitudeIndex = 0;
        int latitudeIndex = 0;

        // Reader for first record of "BBBikeOSMExtract.opl"
        try {
            BufferedReader osmOplExtractReader = new BufferedReader(new FileReader(osmOplExtractFilePath));

            // Find relevant indices using the first OPL data record
            String[] firstOsmOplRecordArray = osmOplExtractReader.readLine().split(" ");
            nodeIdIndex = findIndexInArray("n", firstOsmOplRecordArray); // First object is a node
            longitudeIndex = findIndexInArray("x", firstOsmOplRecordArray);
            latitudeIndex = findIndexInArray("y", firstOsmOplRecordArray);

        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check input file: " + osmOplExtractFilePath);
        }

        try {
            // Reader for body of "BBBikeOSMExtract.opl" (including the first record)
            BufferedReader osmOplExtractReader = new BufferedReader(new FileReader(osmOplExtractFilePath));
            String newline;

            // Read body and process data for all nodes in the road network
            while ((newline = osmOplExtractReader.readLine()) != null) {
                if (newline.substring(0, 1).equalsIgnoreCase("n")) {
                    String[] nodeDataRecord = newline.split(" ");
                    // Example of a node's data record: n127290 v0 dV c0 t i0 u T x11.3246338 y48.2164498
                    long nodeId = Long.parseLong(nodeDataRecord[nodeIdIndex].substring(1));

                    if (this.nodes.containsKey(nodeId)) {
                        double nodeLongitude = Double.parseDouble(nodeDataRecord[longitudeIndex].
                                substring(1));
                        double nodeLatitude = Double.parseDouble(nodeDataRecord[latitudeIndex].
                                substring(1));
                        this.nodes.get(nodeId).setNodeLongitude(nodeLongitude);
                        this.nodes.get(nodeId).setNodeLatitude(nodeLatitude);
                        this.nodes.get(nodeId).setNodeId(nodeId);
                    }
                }
            }
            System.out.println("Nodes' data read from " + osmOplExtractFilePath);

        } catch (FileNotFoundException fNFE) {
            System.out.println("File not found at specified path: " + osmOplExtractFilePath);
        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check input file: " + osmOplExtractFilePath);
        }
    }

    // Associate links with respective nodes
    public void associateLinksWithNode() {
        for (HashMap.Entry<Long, Link> linkEntry : this.links.entrySet()) {
            long linkId = linkEntry.getKey();
            long firstNodeId = linkEntry.getValue().getFirstNodeId();
            long secondNodeId = linkEntry.getValue().getSecondNodeId();

            if (this.nodes.containsKey(firstNodeId)) {
                this.nodes.get(firstNodeId).getLinkIdList().add(linkId);
            }

            if (this.nodes.containsKey(secondNodeId)) {
                this.nodes.get(secondNodeId).getLinkIdList().add(linkId);
            }
        }
        System.out.println("Links associated with respective nodes");
    }

    // Calculate lengths for all links
    public void calculateLinkLengthsM() {
        for (Link link : this.links.values()) {
            Node firstNode = this.nodes.get(link.getFirstNodeId());
            Node secondNode = this.nodes.get(link.getSecondNodeId());

            double secondNodeLongitude = secondNode.getNodeLongitude();
            double secondNodeLatitude = secondNode.getNodeLatitude();
            double linkLengthM = firstNode.equiRectangularDistanceTo(secondNodeLongitude, secondNodeLatitude);

            link.setLinkLengthM(linkLengthM);
        }

        ArrayList<HashMap.Entry<Long, Link>> linksList = new ArrayList<>(this.links.entrySet());
        for (HashMap.Entry<Long, Link> linkEntry : linksList) {
            Link link = linkEntry.getValue();
            if ((link.getlinkLengthM() == 0) || (link.getLinkType() == null)) {
                this.links.remove(linkEntry.getKey());
            }
        }
        System.out.println("Link-wise lengths (in meters) calculated, and zero-cost links deleted");
    }

    /**
     * All writers below are for datasets pertinent to the Dijkstra algorithm, aligned with Dijkstra terminologies
     */

    // Write a "dijkstraLinks.txt" file
    public void writeDijkstraLinks(String dijkstraLinksFilePath) {
        try {
            // Writer for "dijkstraLinks.txt"
            BufferedWriter dijkstraLinksWriter = new BufferedWriter(new FileWriter(dijkstraLinksFilePath));

            // Set up header array
            dijkstraLinksWriter.write("link_id,link_type,first_node_id,second_node_id,link_travel_time_min\n");

            // Write body based on "links" hashmap
            for (HashMap.Entry<Long, Link> linkEntry : this.links.entrySet()) {
                long linkId = linkEntry.getKey();
                String linkType = linkEntry.getValue().getLinkType();
                long firstNodeId = linkEntry.getValue().getFirstNodeId();
                long secondNodeId = linkEntry.getValue().getSecondNodeId();
                double linkTravelTimeMin = linkEntry.getValue().getlinkLengthM();

                dijkstraLinksWriter.write(linkId + "," + linkType + "," + firstNodeId + "," + secondNodeId + "," +
                        linkTravelTimeMin + "\n");
            }
            System.out.println("Links' data written to " + dijkstraLinksFilePath);

        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check the \"links\" hashmap.");
        }
    }

    // Write a "dijkstraNodes.txt" file
    public void writeDijkstraNodes(String dijkstraNodesFilePath) {
        try {
            // Writer for "dijkstraNodes.txt"
            BufferedWriter dijkstraNodesWriter = new BufferedWriter(new FileWriter(dijkstraNodesFilePath));

            // Set up header array
            dijkstraNodesWriter.write("node_id,node_longitude,node_latitude,associated_link_id\n");

            // Write body based on "nodes" hashmap
            for (HashMap.Entry<Long, Node> nodeEntry : this.nodes.entrySet()) {
                long nodeId = nodeEntry.getKey();
                double nodeLongitude = nodeEntry.getValue().getNodeLongitude();
                double nodeLatitude = nodeEntry.getValue().getNodeLatitude();
                for (long associatedLinkId : nodeEntry.getValue().getLinkIdList()) {
                    dijkstraNodesWriter.write(nodeId + "," + nodeLongitude + "," + nodeLatitude + "," +
                            associatedLinkId + "\n");
                }
            }
            System.out.println("Nodes' data written to " + dijkstraNodesFilePath);

        } catch (IOException iOE) {
            System.out.println("Input-output exception. Please check the \"nodes\" hashmap.");
        }
    }

    /**
     * All supporting methods are below
     */

    // Index finder based on presence of certain alphabets in an array element
    private int findIndexInArray(String characterSequenceToFind, String[] headerArray) {
        int columnPosition = -1;
        for (int i = 0; i < headerArray.length; i++) {
            if (headerArray[i].contains(characterSequenceToFind)) {
                columnPosition = i;
                break;
            }
        }
        return columnPosition;
    }

    // Getters of road network data for the Dijkstra algorithm
    public LinkedHashMap<Long, Link> getLinks() {
        return this.links;
    }

    public LinkedHashMap<Long, Node> getNodes() {
        return this.nodes;
    }
}