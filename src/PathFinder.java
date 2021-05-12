import java.awt.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.List;

public class PathFinder {

    private String[][] map;
    private Point robotLocation;

    public PathFinder(String mapPath) throws FileNotFoundException {
        setMap(mapPath);
        setRobotLocation();
    }

    public String[][] getMap() {
        return map;
    }

    public void setMap(String mapPath) throws FileNotFoundException {
        Scanner scanner = new Scanner(new File(mapPath));
        int[] dimension = Arrays.stream(scanner.nextLine().split("\t")).mapToInt(Integer::parseInt).toArray();
        map = new String[dimension[0]][dimension[1]];
        int counter = 0;
        while (scanner.hasNextLine())
            map[counter++] = scanner.nextLine().split("\t");
    }

    public void printMap() {
        for (String[] strings : map) {
            for (int j = 0; j < map[0].length; j++) {
                System.out.print(strings[j] + "\t");
            }
            System.out.println();
        }
        System.out.println();
    }

    private void writeOutput(List<Point> path) throws IOException {
        File output = new File("output.txt");
        StringBuilder stringBuilder = new StringBuilder();
        int cost = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            cost += map[path.get(i).x][path.get(i).y].charAt(0) - '0';
            if (path.get(i).x == path.get(i + 1).x) {
                if (path.get(i).y - path.get(i + 1).y == 1) {
                    stringBuilder.append('L');
                }else {
                    stringBuilder.append('R');
                }
                stringBuilder.append(" ");
            }else {
                if (path.get(i).x - path.get(i + 1).x == 1) {
                    stringBuilder.append('U');
                }else {
                    stringBuilder.append('D');
                }
                stringBuilder.append(" ");
            }
        }
        cost += map[path.get(path.size() - 1).x][path.get(path.size() - 1).y].charAt(0) - '0';
        FileWriter fileWriter = new FileWriter(output);
        fileWriter.write(stringBuilder.toString() + "\n");
        fileWriter.write(cost + "\n");
        fileWriter.write(path.size() + "");
        fileWriter.close();
    }


    public void findRoute() throws IOException {
        while (true) {
            var nearestButter = findNearestButter();
            if (nearestButter == null)
                break;
            var path = robotPath(aStar(nearestButter));
            path.forEach(System.out::println);
            System.out.println("################################");
            writeOutput(path);
        }
    }

    
    private void setRobotLocation() {
        for (int i = 0; i < map.length; i++) {
            for (int j = 0; j < map[0].length; j++) {
                if (map[i][j].contains("r"))
                    robotLocation = new Point(i, j);
            }
        }
    }

    public Point findNearestButter() {
        var explored = new int[map.length][map[0].length];
        ArrayList<Point> frontier = new ArrayList<>();
        frontier.add(robotLocation);
        while (!frontier.isEmpty()) {
            Point nearestButter = frontier.remove(0);
            explored[nearestButter.x][nearestButter.y] = 1;
            if (map[nearestButter.x][nearestButter.y].contains("b"))
                return nearestButter;
            var neighbors = neighborProduction(nearestButter);
            for (Point p : neighbors) {
                if (explored[p.x][p.y] == 0) {
                    frontier.add(p);
                    explored[p.x][p.y] = -1;
                }
            }
        }
        return null;
    }

    private boolean checkNeighbor(Point statusPoint) {
        return statusPoint.x >= 0 && statusPoint.x < map.length &&
                statusPoint.y >= 0 && statusPoint.y < map[0].length &&
                !map[statusPoint.x][statusPoint.y].contains("x") &&
                !map[statusPoint.x][statusPoint.y].contains("p") &&
                !map[statusPoint.x][statusPoint.y].contains("r");
    }

    private List<Point> neighborProduction(Point statusPoint) {
        var result = new ArrayList<Point>();
        Point translate = new Point(statusPoint);
        translate.translate(0, 1);
        if (checkNeighbor(translate))
            result.add(new Point(translate));
        translate.translate(0, -2);
        if (checkNeighbor(translate))
            result.add(new Point(translate));
        translate.translate(1, 1);
        if (checkNeighbor(translate))
            result.add(new Point(translate));
        translate.translate(-2, 0);
        if (checkNeighbor(translate))
            result.add(new Point(translate));
        return result;
    }

    public List<Point> aStar(Point butter) {
        Point saveStatusButter = new Point(butter.x, butter.y);
        var frontier = new PriorityQueue<Node>();
        int[][] mapInfo = new int[map.length][map[0].length];
        boolean findPath = false;
        var dest = nearestPoint(butter, "p");
        frontier.add(new Node(butter, null, manhattanDistance(butter, dest)));
        Node node = null;
        while (!frontier.isEmpty()) {
            node = frontier.poll();
            map[butter.x][butter.y] = map[butter.x][butter.y].charAt(0) + "";
            butter = node.point;
            mapInfo[butter.x][butter.y] = 1;
            if (isGoal(butter)) {
                findPath = true;
                map[butter.x][butter.y] = map[butter.x][butter.y].charAt(0) + "p";
                break;
            }
            updateFrontier(butter, dest, (PriorityQueue<Node>) frontier, mapInfo, node);
        }
        map[saveStatusButter.x][saveStatusButter.y] = map[saveStatusButter.x][saveStatusButter.y].charAt(0) + "b";
        if (!findPath)
            return null;
        var path = new LinkedList<Point>();
        while (node != null) {
            path.addFirst(node.point);
            node = node.parent;
        }
        return path;
    }

    public List<Point> aStar(Point butter, Point dest) {
        var frontier = new PriorityQueue<Node>();
        int[][] mapInfo = new int[map.length][map[0].length];
        boolean findPath = false;
        frontier.add(new Node(butter, null, manhattanDistance(butter, dest)));
        Node node = null;
        while (!frontier.isEmpty()) {
            node = frontier.poll();
            butter = node.point;
            mapInfo[butter.x][butter.y] = 1;
            if (butter.equals(dest)) {
                findPath = true;
                break;
            }
            updateRobotFrontier(butter, dest, (PriorityQueue<Node>) frontier, mapInfo, node);
        }
        if (!findPath)
            return null;
        var path = new LinkedList<Point>();
        while (node != null) {
            path.addFirst(node.point);
            node = node.parent;
        }
        return path;
    }

    private void updateRobotFrontier(Point source, Point dest, PriorityQueue<Node> frontier, int[][] mapInfo, Node node) {
        var neighbors = robotNeighbors(source);
        neighbors.stream().filter(c -> mapInfo[c.x][c.y] == 0)
                .forEach(c -> {
                    frontier.add(new Node(c, node, manhattanDistance(c, dest)));
                    mapInfo[c.x][c.y] = -1;
                });
    }

    private void updateFrontier(Point source, Point dest, PriorityQueue<Node> frontier, int[][] mapInfo, Node node) {
        var neighbors = neighbors(source);
        neighbors.stream().filter(c -> mapInfo[c.x][c.y] == 0)
                .forEach(c -> {
                    frontier.add(new Node(c, node, manhattanDistance(c, dest)));
                    mapInfo[c.x][c.y] = -1;
                });
    }

    public List<Point> robotPath(List<Point> butterPath) {
        var robotPath = new LinkedList<Point>();
        robotPath = (LinkedList<Point>) aStar(robotLocation, findBehind(butterPath.get(0), butterPath.get(1)));
        robotPath.add(butterPath.get(0));
        map[robotLocation.x][robotLocation.y] = map[robotLocation.x][robotLocation.y].charAt(0) + "";
        robotLocation = robotPath.get(robotPath.size() - 1);
        map[robotLocation.x][robotLocation.y] = map[butterPath.get(0).x][butterPath.get(0).y].charAt(0) + "r";
        map[butterPath.get(1).x][butterPath.get(1).y] = map[butterPath.get(1).x][butterPath.get(1).y].charAt(0) + "p";
        for (int i = 1; i < butterPath.size() - 1; i++) {
            map[butterPath.get(i+1).x][butterPath.get(i+1).y] = map[butterPath.get(i+1).x][butterPath.get(i+1).y].charAt(0) + "p";
            var robotStep = aStar(robotLocation, findBehind(butterPath.get(i), butterPath.get(i + 1)));
            robotStep.remove(0);
            robotPath.addAll(robotStep);
            robotPath.add(butterPath.get(i));
            map[robotLocation.x][robotLocation.y] = map[robotLocation.x][robotLocation.y].charAt(0) + "";
            robotLocation = robotPath.get(robotPath.size() - 1);
            map[robotLocation.x][robotLocation.y] = map[butterPath.get(i).x][butterPath.get(i).y].charAt(0) + "r";
        }
        map[butterPath.get(butterPath.size() - 1).x][butterPath.get(butterPath.size() - 1).y] = "pb";
        map[robotLocation.x][robotLocation.y] = map[robotLocation.x][robotLocation.y].charAt(0) + "";
        robotLocation = robotPath.get(robotPath.size() - 1);
        map[robotLocation.x][robotLocation.y] = map[robotLocation.x][robotLocation.y].charAt(0) + "r";
        return robotPath;
    }

    private Point findBehind(Point from, Point to) {
        if (from.x == to.x) {
            return new Point(from.x, 2 * from.y - to.y);
        }else {
            return new Point(2 * from.x - to.x, from.y);
        }
    }

    private boolean isGoal(Point point) {
        return map[point.x][point.y].contains("p") && !map[point.x][point.y].contains("b");
    }

    private List<Point> neighbors(Point point) {
        var result = new ArrayList<Point>();
        var translate = new Point(point);
        translate.translate(1, 0);
        if (isValidLocation(new Point(translate), 'D'))
            result.add(new Point(translate));
        translate.translate(-2, 0);
        if (isValidLocation(new Point(translate), 'U'))
            result.add(new Point(translate));
        translate.translate(1, 1);
        if (isValidLocation(new Point(translate), 'R'))
            result.add(new Point(translate));
        translate.translate(0, -2);
        if (isValidLocation(new Point(translate), 'L'))
            result.add(new Point(translate));
        return result;
    }

    private List<Point> robotNeighbors(Point point) {
        var result = new ArrayList<Point>();
        var translate = new Point(point);
        translate.translate(1, 0);
        if (isValidLocation(new Point(translate)))
            result.add(new Point(translate));
        translate.translate(-2, 0);
        if (isValidLocation(new Point(translate)))
            result.add(new Point(translate));
        translate.translate(1, 1);
        if (isValidLocation(new Point(translate)))
            result.add(new Point(translate));
        translate.translate(0, -2);
        if (isValidLocation(new Point(translate)))
            result.add(new Point(translate));
        return result;
    }

    private boolean isValidLocation(Point point, char direction) {
        return point.x >= 0 && point.x < map.length
                && point.y >= 0 && point.y < map[0].length
                && !map[point.x][point.y].contains("x")
                && !map[point.x][point.y].contains("b")
                && !map[point.x][point.y].contains("r")
                && checkDirection(new Point(point), direction);
    }

    private boolean isValidLocation(Point point) {
        return point.x >= 0 && point.x < map.length
                && point.y >= 0 && point.y < map[0].length
                && !map[point.x][point.y].contains("x")
                && !map[point.x][point.y].contains("b")
                && !map[point.x][point.y].contains("r");
    }

    private boolean checkDirection(Point point, char direction) {
        if (direction == 'R') {
            point.translate(0, -2);
            return isValidLocation(new Point(point));
        } else if (direction == 'L') {
            point.translate(0, 2);
            return isValidLocation(new Point(point));
        } else if (direction == 'U') {
            point.translate(2, 0);
            return isValidLocation(new Point(point));
        }else if (direction == 'D') {
            point.translate(-2, 0);
            return isValidLocation(new Point(point));
        }
        return false;
    }

    private Point nearestPoint(Point source, String destChar) {
        Point tempPoint, dest = null;
        int temp;
        int minManhattan = Integer.MAX_VALUE;
        for (int i = 0; i < map.length; i++)
            for (int j = 0; j < map[0].length; j++)
                if (map[i][j].contains(destChar) && minManhattan > (temp = manhattanDistance(source, tempPoint = new Point(i, j)))) {
                    minManhattan = temp;
                    dest = tempPoint;
                }
        return dest;
    }

    private int manhattanDistance(Point source, Point dest) {
        return Math.abs(source.x - dest.x) + Math.abs(source.y - dest.y);
    }

    public Point getRobotLocation() {
        return robotLocation;
    }

    private class Node implements Comparable<Node> {
        private final Point point;
        private int g, h;
        private final Node parent;
        public Node(Point point, Node parent, int h) {
            this.point = point;
            setG(parent);
            this.h = h;
            this.parent = parent;
        }

        private void setG(Node parent) {
            if (parent == null) {
                this.g = 0;
            } else {
                this.g = parent.g + (map[point.x][point.y].charAt(0) - '0');
            }
        }

        @Override
        public int compareTo(Node o) {
            return Integer.compare(g + h, o.g + o.h);
        }
    }
}
