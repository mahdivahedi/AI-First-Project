import java.awt.*;
import java.io.*;
import java.util.*;
import java.util.List;
import java.util.stream.Stream;

public class PathFinder {

    private int min;
    private int i_source, j_source;
    private ArrayList<Pair> optimalPath;
    private boolean[][] visited;
    private int[] dimension;
    private String[][] map;
    private Point robotLocation;

    public PathFinder(String mapPath) throws FileNotFoundException {
        min = Integer.MAX_VALUE;
        optimalPath = new ArrayList<>();
        setMap(mapPath);
        setRobotLocation();
    }

    public String[][] getMap() {
        return map;
    }

    public void setMap(String mapPath) throws FileNotFoundException {
        Scanner scanner = new Scanner(new File(mapPath));
        dimension = Arrays.stream(scanner.nextLine().split("\t")).mapToInt(Integer::parseInt).toArray();
        map = new String[dimension[0]][dimension[1]];
        int counter = 0;
        while (scanner.hasNextLine())
            map[counter++] = scanner.nextLine().split("\t");
    }

    public void printMap() {
        for (int i = 0; i < map.length; i++) {
            for (int j = 0; j < map[0].length; j++) {
                System.out.print(map[i][j] + "\t");
            }
            System.out.println();
        }
        System.out.println();
    }


    public void findRoute() {
        while (true) {
            var nearestButter = findNearestButter();
            if (nearestButter == null)
                break;
            var path = robotPath(aStar(nearestButter));
            path.forEach(System.out::println);
            System.out.println("################################");
        }
    }


    private void setRobotLocation() {
        for (int i = 0; i < map.length; i++) {
            for (int j = 0; j < map[0].length; j++) {
                if (map[i][j].contains("r")) {
                    robotLocation = new Point(i, j);
                    i_source = i;
                    j_source = j;
                }
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
            if (map[nearestButter.x][nearestButter.y].contains("b") && !map[nearestButter.x][nearestButter.y].contains("n"))
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
//        return nearestPoint(robotLocation, "b");
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
        var dest = nearestDestPointForButter(butter, "p");
        frontier.add(new Node(butter, null, manhattanDistance(butter, dest)));
        Node node = null;
        while (!frontier.isEmpty()) {
//            printMap();
            node = frontier.poll();
            map[butter.x][butter.y] = map[butter.x][butter.y].charAt(0) + "";
            butter = node.point;
            mapInfo[butter.x][butter.y] = 1;
            if (isGoal(butter)) {
                findPath = true;
                map[butter.x][butter.y] = map[butter.x][butter.y].charAt(0) + "p";
                break;
            }
//            map[butter.x][butter.y] = map[butter.x][butter.y].charAt(0) + "b";
            updateFrontier(butter, dest, (PriorityQueue<Node>) frontier, mapInfo, node);
        }
//        map[butter.x][butter.y] = map[butter.x][butter.y].charAt(0) + "";
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
//        robotPath.add(butterPath.get(butterPath.size() - 2));
        map[butterPath.get(butterPath.size() - 1).x][butterPath.get(butterPath.size() - 1).y] = "pb";
        map[robotLocation.x][robotLocation.y] = map[robotLocation.x][robotLocation.y].charAt(0) + "";
        robotLocation = robotPath.get(robotPath.size() - 1);
        map[robotLocation.x][robotLocation.y] = map[robotLocation.x][robotLocation.y].charAt(0) + "r";
        return robotPath;
    }

    // check push from where between to argument
    private Point findBehind(Point from, Point to) {
        if (from.x == to.x) {
            return new Point(from.x, 2 * from.y - to.y);
        } else {
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
        } else if (direction == 'D') {
            point.translate(-2, 0);
            return isValidLocation(new Point(point));
        }
        return false;
    }

    // nearest destination for butter
    private Point nearestDestPointForButter(Point source, String destChar) {
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

    void iterativeDeepeningSearch() {

        visited = new boolean[dimension[0]][dimension[1]];
        // Creating Source and Destination States
        Point destPoint = nearestDestPointForButter(new Point(i_source, j_source), "p");
        State dest = new State(destPoint.x, destPoint.y, 0, dimension[0], dimension[1], visited);

        visited[i_source][j_source] = true;
        State source = new State(i_source, j_source, 1, dimension[0], dimension[1], visited);

        // Repeating until limit found
        for (int limit = 0; limit < (dimension[0] * dimension[1] - 1); limit++) {
            for (int x = 0; x < dimension[0]; x++)
                for (int y = 0; y < dimension[1]; y++) source.visited[x][y] = false;

            // Depth Limited Search
            depthLimitedSearch(source, dest, limit, dimension[0], dimension[1]);
        }

        optimalPath.forEach(p -> System.out.print(("->[" + p.first + "," + p.second + "]")));

    }

    // Private Function for Iterative Deepening Algorithm
    private void depthLimitedSearch(State current, State dest, int limit, int n, int m) {
        int i = current.first;
        int j = current.second;

        // If values out of bound
        if (limit <= 0 || i < 0 || j < 0 || i >= n || j >= m)
            return;

        // Goal State Test
        if (i == dest.first && j == dest.second) {
            if (current.cost < min) {
                min = current.cost;
                optimalPath = current.path;

                optimalPath.add(new Pair(i, j));
            }

            return;
        }

        // Horizontal and Vertical Conditional Search
        if (i - 1 >= 0 && !current.visited[i - 1][j] && !map[i - 1][j].equals("x")) {
            current.visited[i - 1][j] = true;
            State next = new State(i - 1, j, current.cost + 1, n, m, current.visited);
            next.AddPair(current.path, i, j);
            depthLimitedSearch(next, dest, limit - 1, n, m);
        }
        if (i + 1 < n && !current.visited[i + 1][j] && !map[i + 1][j].equals("x")) {
            current.visited[i + 1][j] = true;
            State next = new State(i + 1, j, current.cost + 1, n, m, current.visited);
            next.AddPair(current.path, i, j);
            depthLimitedSearch(next, dest, limit - 1, n, m);
        }
        if (j - 1 >= 0 && !current.visited[i][j - 1] && !map[i][j - 1].equals("x")) {
            current.visited[i][j - 1] = true;
            State next = new State(i, j - 1, current.cost + 1, n, m, current.visited);
            next.AddPair(current.path, i, j);
            depthLimitedSearch(next, dest, limit - 1, n, m);
        }
        if (j + 1 < m && !current.visited[i][j + 1] && !map[i][j + 1].equals("x")) {
            current.visited[i][j + 1] = true;
            State next = new State(i, j + 1, current.cost + 1, n, m, current.visited);
            next.AddPair(current.path, i, j);
            depthLimitedSearch(next, dest, limit - 1, n, m);
        }

/*      // Diagonal and Anti-Diagonal Conditional Search
        if (i - 1 >= 0 && j - 1 >= 0 && !current.visited[i - 1][j - 1] && !map[i - 1][j - 1].equals("x")) {
            current.visited[i - 1][j - 1] = true;
            State next = new State(i - 1, j - 1, current.cost + 1, n, m, current.visited);
            next.AddPair(current.path, i, j);
            depthLimitedSearch(next, dest, limit - 1, n, m);
        }
        if (i - 1 >= 0 && j + 1 < m && !current.visited[i - 1][j + 1] && !map[i - 1][j + 1].equals("x")) {
            current.visited[i - 1][j + 1] = true;
            State next = new State(i - 1, j + 1, current.cost + 1, n, m, current.visited);
            next.AddPair(current.path, i, j);
            depthLimitedSearch(next, dest, limit - 1, n, m);
        }
        if (i + 1 < n && j - 1 >= 0 && !current.visited[i + 1][j - 1] && !map[i + 1][j - 1].equals("x")) {
            current.visited[i + 1][j - 1] = true;
            State next = new State(i + 1, j - 1, current.cost + 1, n, m, current.visited);
            next.AddPair(current.path, i, j);
            depthLimitedSearch(next, dest, limit - 1, n, m);
        }
        if (i + 1 < n && j + 1 < m && !current.visited[i + 1][j + 1] && !map[i + 1][j + 1].equals("x")) {
            current.visited[i + 1][j + 1] = true;
            State next = new State(i + 1, j + 1, current.cost + 1, n, m, current.visited);
            next.AddPair(current.path, i, j);
            depthLimitedSearch(next, dest, limit - 1, n, m);
        }
*/
        // If nothing is searches on this level
    }

    void printResult() {
        System.out.println("\n\nIterative Deepening Search Result!");
        System.out.print("Optimal Path Cost: " + optimalPath.size() + "\nOptimal Path: {");
        for (int i = 0; i < optimalPath.size(); i++) {
            System.out.print("(" + optimalPath.get(i).first + ", " + optimalPath.get(i).second + ")");
            if (i != optimalPath.size() - 1) System.out.print(" ");
        }
        System.out.println("}");
    } // #EndOfPrintResult!

    void writeFile() throws IOException {
        File file = new File("Output/IDS-Result.txt");

        if (!file.exists())
            file.createNewFile();

        FileWriter fw = new FileWriter(file);
        BufferedWriter bw = new BufferedWriter(fw);

        bw.write("Iterative Deepening Search Result!");
        bw.newLine();
        bw.write("Optimal Path Cost: " + optimalPath.size());
        bw.newLine();
        bw.write("Optimal Path: {");
        for (int i = 0; i < optimalPath.size(); i++) {
            bw.write("(" + optimalPath.get(i).first + ", " + optimalPath.get(i).second + ")");
            if (i != optimalPath.size() - 1) bw.write(" ");
        }
        bw.write("}");
        bw.newLine();

        bw.flush();
        bw.close();
        fw.close();
    } // #EndOfWriteFile!

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
