package frc.robot.util;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

public class WLEDController {

    // --- 1. DEFINE YOUR PRESETS HERE ---
    // This allows the team to use names instead of random numbers
    public enum State {
        OFF(0),
        IDLE(1),
        NO_TAG(2),
        HAS_TAG(3);

        public final int id;
        State(int id) { this.id = id; }
    }

    private final HttpClient client;
    private final String url;

    public WLEDController(String ipAddress) {
        this.url = "http://" + ipAddress + "/json/state";
        this.client = HttpClient.newBuilder()
                .connectTimeout(Duration.ofSeconds(2))
                .build();
    }

    /**
     * The main method the team will use. 
     * Sets the lights based on the Robot State.
     */
    public void set(State state) {
        if (state == State.OFF) {
            sendJson("{\"on\":false}");
        } else {
            sendJson("{\"on\":true,\"ps\":" + state.id + "}");
        }
    }

    private void sendJson(String json) {
        HttpRequest request = HttpRequest.newBuilder()
                .uri(URI.create(url))
                .header("Content-Type", "application/json")
                .POST(HttpRequest.BodyPublishers.ofString(json))
                .build();

        // sendAsync ensures the robot doesn't stutter if the network is slow
        client.sendAsync(request, HttpResponse.BodyHandlers.discarding());
    }
}