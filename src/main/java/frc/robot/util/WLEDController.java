package frc.robot.util;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;

public class WLEDController {
  private final HttpClient client = HttpClient.newHttpClient();
  private final String url;

  public WLEDController(String ip) {
    this.url = "http://" + ip + "/json/state";
  }

  public void setPreset(int id) {
    send("{\"ps\":" + id + "}");
  }

  public void setPower(boolean on) {
    send("{\"on\":" + on + "}");
  }

  private void send(String json) {
    HttpRequest request =
        HttpRequest.newBuilder()
            .uri(URI.create(url))
            .header("Content-Type", "application/json")
            .POST(HttpRequest.BodyPublishers.ofString(json))
            .build();
    client.sendAsync(request, HttpResponse.BodyHandlers.discarding());
  }
}
