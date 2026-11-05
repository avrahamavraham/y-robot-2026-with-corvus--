package frc.lib.PathFinding;

import eth.epieffe.jwalker.Edge;
import eth.epieffe.jwalker.Graph;
import java.util.ArrayList;
import java.util.List;


public class Grid implements Graph<Cell> {

    private static final String[] DIR_LABEL = {"LEFT", "RIGHT", "UP", "DOWN"};
    private static final int[] DIR_ROW = {0, 0, -1, 1};
    private static final int[] DIR_COL = {-1, 1, 0, 0};

    private final boolean[][] grid; // true = walkable, false = obstacle

    public Grid(boolean[][] grid) { this.grid = grid; }

    @Override
    public List<Edge<Cell>> outgoingEdges(Cell cell) {
        List<Edge<Cell>> edges = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            int r = cell.row() + DIR_ROW[i];
            int c = cell.col() + DIR_COL[i];
            if (r < 0 || r >= grid.length || c < 0 || c >= grid[r].length) continue;
            if (grid[r][c]) edges.add(new Edge<>(new Cell(r, c), 1, DIR_LABEL[i]));
        }
        return edges;
    }
}