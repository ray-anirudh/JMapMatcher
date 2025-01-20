package GPSDataManager;

import java.io.*;
import java.util.Arrays;
import java.util.LinkedHashMap;

public class GPSNodeReaderWriter {
    private final LinkedHashMap<Long, GPSNode> gPSNodes = new LinkedHashMap<>();

    // Read and store all the GPS nodes in the aforementioned hashmap
    public void readGPSNodes (String gPSNodesFilePath) {
        try{
            // Create a reader for the GPS nodes file
            BufferedReader gPSNodesReader = new BufferedReader(new FileReader(gPSNodesFilePath));
            String newline;

            // Find pertinent indices
            String[] gPSNodesHeaderArray = gPSNodesReader.readLine().trim().split(",");
                    // .replaceAll("[^\\x20-\\x7E]", "") // Remove non-printable ASCII characters
                    // .split(",");
            int gPSNodeIdIndex = findIndexInArray(gPSNodesHeaderArray, "ID");
            int gPSNodeLongitudeIndex = findIndexInArray(gPSNodesHeaderArray, "Longitude");
            int gPSNodeLatitudeIndex = findIndexInArray(gPSNodesHeaderArray, "Latitude");
            int gPSNodeDateTimeStampIndex = findIndexInArray(gPSNodesHeaderArray, "Date.Time");
            int gPSNodeAtDepotIndex = findIndexInArray(gPSNodesHeaderArray, "atDepot");
            int gPSNodeAtDepotLagIndex = findIndexInArray(gPSNodesHeaderArray, "atDepot_lag");
            int gPSNodeAtDepotNextIndex = findIndexInArray(gPSNodesHeaderArray, "atDepot_next");
            int gPSNodeHubStayIndex = findIndexInArray(gPSNodesHeaderArray, "hubStay");
            int gPSNodeHubStartIndex = findIndexInArray(gPSNodesHeaderArray, "hubStart");
            int gPSNodeHubStopIndex = findIndexInArray(gPSNodesHeaderArray, "hubStop");
            int gPSNodeTourNumberIndex = findIndexInArray(gPSNodesHeaderArray, "Tour");
            int gPSNodeIsStopIndex = findIndexInArray(gPSNodesHeaderArray, "isStop");

            // Read body and process data
            while ((newline = gPSNodesReader.readLine()) != null) {
                String[] gPSNodeDataRecord = newline.split(",");
                // System.out.println(Arrays.toString(gPSNodeDataRecord));  // Debugging support
                long gPSNodeId = Long.parseLong(gPSNodeDataRecord[gPSNodeIdIndex]);
                double gPSNodeLongitude = Double.parseDouble(gPSNodeDataRecord[gPSNodeLongitudeIndex]);
                double gPSNodeLatitude = Double.parseDouble(gPSNodeDataRecord[gPSNodeLatitudeIndex]);
                String gPSNodeDateTimeStamp = gPSNodeDataRecord[gPSNodeDateTimeStampIndex];
                String gPSNodeAtDepot = gPSNodeDataRecord[gPSNodeAtDepotIndex];
                String gPSNodeAtDepotLag = gPSNodeDataRecord[gPSNodeAtDepotLagIndex];
                String gPSNodeAtDepotNext = gPSNodeDataRecord[gPSNodeAtDepotNextIndex];
                String gPSNodeHubStay = gPSNodeDataRecord[gPSNodeHubStayIndex];
                String gPSNodeHubStart = gPSNodeDataRecord[gPSNodeHubStartIndex];
                String gPSNodeHubStop = gPSNodeDataRecord[gPSNodeHubStopIndex];
                int gPSNodeTourNumber = Integer.parseInt(gPSNodeDataRecord[gPSNodeTourNumberIndex]);
                String gPSNodeIsStop = gPSNodeDataRecord[gPSNodeIsStopIndex];

                GPSNode gPSNode = new GPSNode(gPSNodeId, gPSNodeLongitude, gPSNodeLatitude, gPSNodeDateTimeStamp,
                        gPSNodeAtDepot, gPSNodeAtDepotLag, gPSNodeAtDepotNext, gPSNodeHubStay, gPSNodeHubStart,
                        gPSNodeHubStop, gPSNodeTourNumber, gPSNodeIsStop);
                this.gPSNodes.put(gPSNodeId, gPSNode);
            }

        } catch (FileNotFoundException fNFE) {
            System.out.println("File not found at " + gPSNodesFilePath);
        } catch (IOException iOE) {
            System.out.println("Input-output exception; please check file " + gPSNodesFilePath);
            iOE.printStackTrace();
        }
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

    public LinkedHashMap<Long, GPSNode> getGPSNodes() {
        return this.gPSNodes;
    }
}