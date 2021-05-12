import java.io.*;

public class IDS {

    public static void main(String[] args) throws FileNotFoundException {
        PathFinder p = new PathFinder("/home/meyti/Documents/AI/Projects/Project_1_AI/src/test1.txt");
        p.iterativeDeepeningSearch();
    }
}
