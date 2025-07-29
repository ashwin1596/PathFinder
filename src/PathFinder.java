import javax.imageio.stream.ImageInputStream;
import javax.imageio.stream.ImageOutputStream;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.*;
import java.util.List;

import static javax.imageio.ImageIO.*;

class Node {
    double x;
    double y;
    double f;
    double elevation;

    Node(double x, double y, double elevation) {
        this.x = x;
        this.y = y;
        this.elevation = elevation;
    }

    public void setF(double f) {
        this.f = f;
    }

    @Override
    public String toString() {
        return String.format("x: %f, y: %f", this.x, this.y);
    }
}

public class PathFinder {
    private static final int numOfArgs = 4;
    private static final String outputFileFormat = "png";

    private static final int width = 395;
    private static final int height = 500;
    private static final double pixel_width = 10.29;
    private static final double pixel_height = 7.55;
    private static final int water_penalty = 100;
    private final List<double[]> controlPoints;
    private String outputFilename;
    private String terrainImage;
    private String pathFile;
    private String elevationFile;
    private Map<List<Double>, Double> gValues;
    private double[] destNode;
    private double[][] elevations;
    private Map<Integer, Boolean> colorMap; // key:color, value:is walkable
    private Map<Node, Node> path; // final path

    PathFinder() {
        this.controlPoints = new ArrayList<>();
        this.gValues = new HashMap<>();
        this.path = new HashMap<>();
        this.elevations = new double[width][height];
        this.colorMap = new HashMap<>();
    }

    void getArgs(String[] args) {
        this.terrainImage = args[0];
        this.elevationFile = args[1];
        this.pathFile = args[2];
        this.outputFilename = args[3];
    }

    public static void main(String[] args) {
        if (args.length != numOfArgs) {
            System.err.println("Number of args mismatch");
            return;
        }

        lab1 obj = new lab1();
        obj.getArgs(args);
        obj.getControlPoints();
        obj.getElevations();
        obj.loadColorMap();
        obj.loadTerrainMap();
    }

    private void getControlPoints() {
        try (BufferedReader br = new BufferedReader(new FileReader(this.pathFile))) {
            String input;
            while ((input = br.readLine()) != null) {
                this.controlPoints.add(Arrays.stream(input.split(" ")).mapToDouble(Double::parseDouble).toArray());
            }

            this.destNode = this.controlPoints.get(1);
        } catch (IOException ex) {
            System.err.printf("Exception caught: %s", ex.getMessage());
        }
    }

    private void getElevations() {
        try (BufferedReader br = new BufferedReader(new FileReader(this.elevationFile))) {
            String input;
            int row = 0;
            while ((input = br.readLine()) != null) {
                String[] ele = input.strip().split("\\s+");
                for (int i = 0; i < ele.length - 5; i++) {
                    String[] splitValues = ele[i].split("e+");
                    this.elevations[i][row] = Double.parseDouble(splitValues[0]) * Math.pow(10, Integer.parseInt(splitValues[1]));
                }
                row++;
            }
        } catch (IOException ex) {
            System.err.printf("Exception caught: %s", ex.getMessage());
        }
    }

    private void loadTerrainMap() {

        try (
                ImageInputStream inputStream = createImageInputStream(new FileInputStream(this.terrainImage));
                ImageOutputStream outputStream = createImageOutputStream(new FileOutputStream(this.outputFilename))
        ) {
            BufferedImage image = read(inputStream);

            int ind = 0;

            while (ind < this.controlPoints.size() - 1) {
                Node startNode = new Node(this.controlPoints.get(ind)[0], this.controlPoints.get(ind)[1], this.elevations[(int) this.controlPoints.get(ind)[0]][(int) this.controlPoints.get(ind)[1]]);
                double[] destNode = new double[]{this.controlPoints.get(ind + 1)[0], this.controlPoints.get(ind + 1)[1]};
                Node node = aStarSearch(startNode, destNode, image);

                while (this.path.get(node).x != -1 && this.path.get(node).y != -1) {
                    image.setRGB((int) node.x, (int) node.y, new Color(200, 100, 230).getRGB());
                    node = this.path.get(node);
                }
                image.setRGB((int) node.x, (int) node.y, new Color(200, 100, 230).getRGB());

                ind++;
            }

            int totalControlPoints = this.controlPoints.size();
            System.out.println(this.gValues.get(Arrays.asList(this.controlPoints.get(totalControlPoints - 1)[0], this.controlPoints.get(totalControlPoints - 1)[1])));

            write(image, outputFileFormat, outputStream);
        } catch (IOException ex) {
        }
    }

    private double getGofN(double gOfPreviousNode, Node currentNode, Node parentNode, double elevation) {
        return gOfPreviousNode + Math.sqrt(Math.pow((currentNode.x - parentNode.x) * pixel_width, 2) + Math.pow((currentNode.y - parentNode.y) * pixel_height, 2) + Math.pow((currentNode.elevation - parentNode.elevation), 2));
    }

    private int getHofN(Node currentNode, double heightTravelled, boolean isWaterLakeMarsh) {
        if (!isWaterLakeMarsh)
            return (int) Math.sqrt(Math.pow(currentNode.x - destNode[0], 2) + Math.pow(currentNode.y - destNode[1], 2)) + (int) heightTravelled;

        return (int) Math.sqrt(Math.pow(currentNode.x - destNode[0], 2) + Math.pow(currentNode.y - destNode[1], 2)) + (int) heightTravelled + water_penalty; //adding 100 for water
    }

    private Node aStarSearch(Node startNode, double[] destNode, BufferedImage image) {
        int[] visited = new int[width * height];
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingDouble(a -> a.f)); //[Pixel position, f(n)]
        double start_g = this.gValues.get(Arrays.asList(startNode.x, startNode.y)) == null ? 0 : this.gValues.get(Arrays.asList(startNode.x, startNode.y));
        if (start_g == 0)
            this.gValues.put(Arrays.asList(startNode.x, startNode.y), 0D);
        startNode.setF(start_g + getHofN(startNode, 0, false));
        this.path = new HashMap<>();
        this.path.put(startNode, new Node(-1, -1, -1));

        queue.offer(startNode);

        while (!queue.isEmpty()) {
            Node node = queue.poll();

            if (visited[(int) node.x + (int) node.y * width] == 1)
                continue;

            visited[(int) node.x + (int) node.y * width] = 1;

            if (!this.colorMap.get(image.getRGB((int) node.x, (int) node.y)))
                continue;

            if (node.x == destNode[0] && node.y == destNode[1]) {
                return node;
            }

            List<Node> neighbours = getNeighbours(node, image);
            for (Node neighbour : neighbours) {
                queue.offer(neighbour);
            }
        }

        return null;
    }

    private List<Node> getNeighbours(Node node, BufferedImage image) {
        Node parentNode = node;
        List<Node> neighbours = new ArrayList<>();

        if (node.x > 0) {
            Node currentNode = new Node(node.x - 1, node.y, this.elevations[(int) node.x - 1][(int) node.y]);
            boolean isWaterLakeMarsh = image.getRGB((int) currentNode.x, (int) currentNode.y) == new Color(0, 0, 255).getRGB();
            double heightTravelled = Math.abs(this.elevations[(int) parentNode.x][(int) parentNode.y] - this.elevations[(int) currentNode.x][(int) currentNode.y]);
            double g = getGofN(this.gValues.get(Arrays.asList(node.x, node.y)), currentNode, parentNode, heightTravelled);
            this.gValues.put(Arrays.asList(node.x - 1, node.y), g);
            double h = getHofN(currentNode, heightTravelled, isWaterLakeMarsh);
            currentNode.setF(g + h);
            neighbours.add(currentNode);
            this.path.put(currentNode, parentNode);
        }
        if (node.x < width - 1) {
            Node currentNode = new Node(node.x + 1, node.y, this.elevations[(int) node.x + 1][(int) node.y]);
            boolean isWaterLakeMarsh = image.getRGB((int) currentNode.x, (int) currentNode.y) == new Color(0, 0, 255).getRGB();
            double heightTravelled = Math.abs(this.elevations[(int) parentNode.x][(int) parentNode.y] - this.elevations[(int) currentNode.x][(int) currentNode.y]);
            double g = getGofN(this.gValues.get(Arrays.asList(node.x, node.y)), currentNode, parentNode, heightTravelled);
            this.gValues.put(Arrays.asList(node.x + 1, node.y), g);
            double h = getHofN(currentNode, heightTravelled, isWaterLakeMarsh);
            currentNode.setF(g + h);
            neighbours.add(currentNode);
            this.path.put(currentNode, parentNode);
        }
        if (node.y > 0) {
            Node currentNode = new Node(node.x, node.y - 1, this.elevations[(int) node.x][(int) node.y - 1]);
            boolean isWaterLakeMarsh = image.getRGB((int) currentNode.x, (int) currentNode.y) == new Color(0, 0, 255).getRGB();
            double heightTravelled = Math.abs(this.elevations[(int) parentNode.x][(int) parentNode.y] - this.elevations[(int) currentNode.x][(int) currentNode.y]);
            double g = getGofN(this.gValues.get(Arrays.asList(node.x, node.y)), currentNode, parentNode, heightTravelled);
            this.gValues.put(Arrays.asList(node.x, node.y - 1), g);
            double h = getHofN(currentNode, heightTravelled, isWaterLakeMarsh);
            currentNode.setF(g + h);
            neighbours.add(currentNode);
            this.path.put(currentNode, parentNode);
        }
        if (node.y < height - 1) {
            Node currentNode = new Node(node.x, node.y + 1, this.elevations[(int) node.x][(int) node.y + 1]);
            boolean isWaterLakeMarsh = image.getRGB((int) currentNode.x, (int) currentNode.y) == new Color(0, 0, 255).getRGB();
            double heightTravelled = Math.abs(this.elevations[(int) parentNode.x][(int) parentNode.y] - this.elevations[(int) currentNode.x][(int) currentNode.y]);
            double g = getGofN(this.gValues.get(Arrays.asList(node.x, node.y)), currentNode, parentNode, heightTravelled);
            this.gValues.put(Arrays.asList(node.x, node.y + 1), g);
            double h = getHofN(currentNode, heightTravelled, isWaterLakeMarsh);
            currentNode.setF(g + h);
            neighbours.add(currentNode);
            this.path.put(currentNode, parentNode);
        }

        return neighbours;
    }

    private void loadColorMap() {
        this.colorMap.put(new Color(248, 148, 18).getRGB(), true);    // "OpenLand"
        this.colorMap.put(new Color(255, 192, 0).getRGB(), true);     // "RoughMeadow"
        this.colorMap.put(new Color(255, 255, 255).getRGB(), true);   // "EasyMovementForest"
        this.colorMap.put(new Color(2, 208, 60).getRGB(), true);      // "SlowRunForest"
        this.colorMap.put(new Color(2, 136, 40).getRGB(), true);      // "WalkForest"
        this.colorMap.put(new Color(5, 73, 24).getRGB(), false);       // "ImpassibleVegetation"
        this.colorMap.put(new Color(0, 0, 255).getRGB(), true);       // "LakeSwampMarsh"
        this.colorMap.put(new Color(71, 51, 3).getRGB(), true);       // "PavedRoad"
        this.colorMap.put(new Color(0, 0, 0).getRGB(), true);         // "FootPath"
        this.colorMap.put(new Color(205, 0, 101).getRGB(), false);     // "OutOfBounds"
        this.colorMap.put(-3644186, true);     // "OutOfBounds"
    }
}