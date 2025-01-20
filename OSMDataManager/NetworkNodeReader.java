package OSMDataManager;

import KDTreeManager.KDTreeNode;
import KDTreeManager.KDTreeBuilderSearcher;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;

public class NetworkNodeReader {
    private final LinkedHashMap<Long, NetworkNode> networkNodes = new LinkedHashMap<>();
    private final LinkedHashMap<Long, ArrayList<Long>> connectivityMatrix = new LinkedHashMap<>();
    private final int NUMBER_NEIGHBOURS_CONNECTIVITY_MATRIX = 70;

    // Read and store all the network nodes in the aforementioned hashmap
    public void readNetworkNodes(String networkNodesFilePath) {
        try {
            // Create a reader for the "nodes.csv" file
            BufferedReader networkNodesReader = new BufferedReader(new FileReader(networkNodesFilePath));
            String newline;

            // Find pertinent indices
            String[] networkNodesHeaderArray = networkNodesReader.readLine().split(",");
            int networkNodeIdIndex = findIndexInArray(networkNodesHeaderArray, "fid");
            int roadIdIndex = findIndexInArray(networkNodesHeaderArray, "osm_id");
            int roadNameIndex = findIndexInArray(networkNodesHeaderArray, "name");
            int networkNodeLongitudeIndex = findIndexInArray(networkNodesHeaderArray, "node_longitude");
            int networkNodeLatitudeIndex = findIndexInArray(networkNodesHeaderArray, "node_latitude");

            // Read body and process data
            while ((newline = networkNodesReader.readLine()) != null) {
                String[] networkNodeDataRecord = newline.split(",");

                long networkNodeId = Long.parseLong(networkNodeDataRecord[networkNodeIdIndex].replaceAll("\"",
                        ""));
                long roadOsmId = Long.parseLong(networkNodeDataRecord[roadIdIndex].replaceAll("\"",
                        ""));
                String roadName = networkNodeDataRecord[roadNameIndex];
                double networkNodeLongitude = Double.parseDouble(networkNodeDataRecord[networkNodeLongitudeIndex]);
                double networkNodeLatitude = Double.parseDouble(networkNodeDataRecord[networkNodeLatitudeIndex]);

                NetworkNode networkNode = new NetworkNode(networkNodeId, roadOsmId, roadName, networkNodeLongitude,
                        networkNodeLatitude);
                this.networkNodes.put(networkNodeId, networkNode);
            }

        } catch (FileNotFoundException fNFE) {
            System.out.println("File not found at " + networkNodesFilePath);
        } catch (IOException iOE) {
            System.out.println("Input-output exception; please check file " + networkNodesFilePath);
            iOE.printStackTrace();
        }
    }

    // Build the road connectivity matrix
    public void buildConnectivityMatrix() {
        long connMatrixStartTime = System.nanoTime();
        NetworkNode[] networkNodes = this.networkNodes.values().toArray(new NetworkNode[0]);

        KDTreeBuilderSearcher kDTreeBuilderSearcher = new KDTreeBuilderSearcher();
        kDTreeBuilderSearcher.buildNodeBasedKDTree(networkNodes);

        for (NetworkNode networkNode : networkNodes) {
            long primaryRoadOsmId = networkNode.getRoadOsmId();
            if (!this.connectivityMatrix.containsKey(primaryRoadOsmId)) {
                ArrayList<Long> roadSpecificConnectivitySet = new ArrayList<>();
                roadSpecificConnectivitySet.add(primaryRoadOsmId);
                this.connectivityMatrix.put(primaryRoadOsmId, roadSpecificConnectivitySet);
            }

            List<NetworkNode> neighbouringNodes = kDTreeBuilderSearcher.findNearestNodes(networkNode.
                            getNetworkNodeLongitude(), networkNode.getNetworkNodeLatitude(),
                    NUMBER_NEIGHBOURS_CONNECTIVITY_MATRIX);
            for (NetworkNode neighbouringNode : neighbouringNodes) {
                this.connectivityMatrix.get(primaryRoadOsmId).add(neighbouringNode.getRoadOsmId());
                // System.out.println(neighbouringNode.toString());    // Debugging statement
            }
        }

        long connMatrixEndTime = System.nanoTime();
        long nsInS = 60_000_000_000L;
        System.out.println("Connectivity matrix built in: " + ((connMatrixEndTime - connMatrixStartTime) / nsInS +
                " minutes."));
    }

    // Read array and find index
    private int findIndexInArray(String[] stringArray, String string) {
        int index = -1;
        for(int i = 0; i < stringArray.length; i++) {
            if(stringArray[i].equalsIgnoreCase(string)) {
                index = i;
                break;
            }
        }
        return index;
    }

    public LinkedHashMap<Long, NetworkNode> getNetworkNodes() {
        return this.networkNodes;
    }

    public LinkedHashMap<Long, ArrayList<Long>> getConnectivityMatrix() {
        return this.connectivityMatrix;
    }
}