import java.util.ArrayList;

public class State {
    int first, second, cost;
    ArrayList<Pair> path;
    boolean[][] visited;
    
    State ( int fi, int se, int c, int n, int m, boolean v[][] ) {
        first = fi;
        second = se;
        cost = c;
        path = new ArrayList<>();
        
        visited = new boolean[n][m];
        for(int i=0; i<n; i++) {
            System.arraycopy(v[i], 0, visited[i], 0, m);
        }
    }
    
    void AddPair ( ArrayList<Pair> visited, int i, int j ) {
        path.addAll(visited);
        path.add(new Pair(i, j));
    }
}
