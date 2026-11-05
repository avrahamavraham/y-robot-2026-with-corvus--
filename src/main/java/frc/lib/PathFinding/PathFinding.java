// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.PathFinding;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import eth.epieffe.jwalker.Edge;
import eth.epieffe.jwalker.Graph;
import eth.epieffe.jwalker.Heuristic;
import eth.epieffe.jwalker.Visit;
import eth.epieffe.jwalker.Visits;
import frc.lib.FieldConstants;
import frc.robot.lib.BLine.BLineField;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;

/** Add your docs here. */
public class PathFinding {
    private NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private StructArrayPublisher<Translation2d> translationPublisher = instance.getStructArrayTopic("transltion2dPathfinding", Translation2d.struct).publish();
    private Graph<Cell> grid;
    private FollowPath.Builder pathBuilder;
    private Subsystem driveTrain;
    private double cellSizeX;
    private double cellSizeY;
    private int rows;
    private int cols;
    public PathFinding(boolean[][] grid,FollowPath.Builder pathBuilder,Subsystem driveTrain){
        this.grid = new Grid(grid);
        this.pathBuilder = pathBuilder;
        this.driveTrain = driveTrain;
        this.rows = grid.length;
        this.cols = grid[0].length;
        this.cellSizeX = FieldConstants.fieldLength/ grid.length;
        this.cellSizeY = FieldConstants.fieldWidth/ grid[0].length;
    }

    private Translation2d toTranslation(Cell cell) {
        return new Translation2d((cell.row() + 0.5) * cellSizeX,
                                (cell.col() + 0.5) * cellSizeY);
    }
    private Cell toCell(Translation2d p) {
        int r = (int) Math.floor(p.getX() / cellSizeX);
        int c = (int) Math.floor(p.getY() / cellSizeY);
        r = Math.max(0, Math.min(rows - 1, r));
        c = Math.max(0, Math.min(cols - 1, c));
        return new Cell(r, c);
    }
    public List<Translation2d> PathFindToPoint(Pose2d robotPose, Pose2d targetPose){
        Cell target = toCell(targetPose.getTranslation());
        Cell start = toCell(robotPose.getTranslation());
        Heuristic<Cell> manhattan = cell ->
        Math.abs(cell.row() - target.row()) + Math.abs(cell.col() - target.col());

        Visit<Cell> aStar = Visits.aStar(grid, manhattan);
        List<Edge<Cell>> path = aStar.run(start);
        if (path == null) {
            return List.of();
        }
        List<Translation2d> translation2dPath = path.stream().map((EdgeCell)-> {
            return toTranslation(EdgeCell.destination);
        }).toList();
        translationPublisher.accept(translation2dPath.toArray(Translation2d[]::new));
        return translation2dPath;
    }

    public Path pathFindPathCreate(Pose2d robotPose, Pose2d targetPose){
        // targetPose = AllianceFlipUtil.apply(targetPose);
        TranslationTarget[] translationTarget = PathFindToPoint(robotPose, targetPose).stream().map((translation)-> new TranslationTarget(translation)).toArray(TranslationTarget[]::new); 
        if (translationTarget.length == 0) {
            return new Path();
        }
        Waypoint startWayPoint = new Waypoint(robotPose);
        Waypoint endWayPoint = new Waypoint(targetPose);
        var path = new Path(startWayPoint);
        for (TranslationTarget translationTarget2 : translationTarget) {
            path.addPathElement(translationTarget2);
        } 
        path.addPathElement(endWayPoint);
        Field2d field = new Field2d();
        SmartDashboard.putData("PathFinding", field);
        BLineField.drawPath(field, path);
        return path;
    }
    public Command PathfindAndFollow(Supplier<Pose2d> robotPose,Pose2d targetPose){
        return Commands.defer(()-> pathBuilder.build(pathFindPathCreate(robotPose.get(), targetPose)), 
        Set.of(driveTrain));
    }

    
    /**
     * @param fileName            path inside the deploy folder, e.g. "pathplanner/navgrid.json"
     * @param robotRadiusMeters   obstacles are grown by this much (0 = no inflation)
     */
    public static boolean[][] load(String fileName, double robotRadiusMeters) {
        File file = new File(Filesystem.getDeployDirectory(), fileName);
        try {
            JsonNode root = new ObjectMapper().readTree(file);
            JsonNode gridNode = root.isArray() ? root : root.get("grid");
            if (gridNode == null || gridNode.size() == 0) {
                throw new IllegalStateException("No grid found in " + file);
            }
 
            int numRows = gridNode.size();            // along Y
            int numCols = gridNode.get(0).size();     // along X
 
            boolean[][] walkable = new boolean[numCols][numRows];
            for (int y = 0; y < numRows; y++) {
                JsonNode row = gridNode.get(y);
                for (int x = 0; x < numCols; x++) {
                    walkable[x][y] = !row.get(x).asBoolean();
                }
            }
 
            return robotRadiusMeters > 0 ? inflate(walkable, robotRadiusMeters) : walkable;
        } catch (IOException e) {
            throw new UncheckedIOException("Could not read nav grid: " + file, e);
        }
    }
 
    private static boolean[][] inflate(boolean[][] walkable, double radius) {
        int cols = walkable.length;      // X cells
        int rows = walkable[0].length;   // Y cells
        double cellX = FieldConstants.fieldLength / cols;
        double cellY = FieldConstants.fieldWidth / rows;
        int rx = (int) Math.ceil(radius / cellX);
        int ry = (int) Math.ceil(radius / cellY);
 
        boolean[][] result = new boolean[cols][];
        for (int x = 0; x < cols; x++) {
            result[x] = walkable[x].clone();
        }
 
        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                if (walkable[x][y]) continue;
                for (int dx = -rx; dx <= rx; dx++) {
                    for (int dy = -ry; dy <= ry; dy++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= cols || ny >= rows) continue;
                        double distX = dx * cellX;
                        double distY = dy * cellY;
                        if (distX * distX + distY * distY <= radius * radius) {
                            result[nx][ny] = false;
                        }
                    }
                }
            }
        }
        return result;
    }
}