const grid = document.getElementById("motor-grid");
const template = document.getElementById("motor-template");

async function fetchMotors() {
  const response = await fetch("/api/motors");
  if (!response.ok) {
    throw new Error(`Failed to load motors: ${response.statusText}`);
  }
  const payload = await response.json();
  return payload.motors ?? [];
}

function renderMotorCard(motor) {
  const node = template.content.firstElementChild.cloneNode(true);
  const title = node.querySelector(".motor-name");
  const idLabel = node.querySelector(".motor-id");
  const stateLabel = node.querySelector(".motor-state");
  const button = node.querySelector(".toggle-btn");

  title.textContent = motor.label;
  idLabel.textContent = motor.id;
  button.dataset.motorId = motor.id;

  updateMotorVisuals(node, motor);

  button.addEventListener("click", async () => {
    try {
      button.disabled = true;
      const toggled = await toggleMotor(motor.id);
      motor.inverted = toggled.inverted;
      updateMotorVisuals(node, motor);
    } catch (error) {
      console.error(error);
      alert(`Failed to toggle ${motor.label}: ${error.message}`);
    } finally {
      button.disabled = false;
    }
  });

  node.dataset.motorId = motor.id;
  return node;
}

function updateMotorVisuals(card, motor) {
  const stateLabel = card.querySelector(".motor-state");
  const button = card.querySelector(".toggle-btn");
  const stateText = motor.inverted ? "Inverted" : "Normal orientation";
  stateLabel.textContent = `Current state: ${stateText}`;
  button.textContent = motor.inverted ? "Set to normal" : "Invert direction";
  button.dataset.inverted = String(motor.inverted);
}

async function toggleMotor(motorId) {
  const response = await fetch(`/api/motors/${encodeURIComponent(motorId)}/toggle`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
  });
  if (!response.ok) {
    const message = await response.text();
    throw new Error(message || "Request failed");
  }
  return response.json();
}

async function bootstrap() {
  try {
    grid.textContent = "Loading motors...";
    const motors = await fetchMotors();
    grid.textContent = "";
    motors.forEach((motor) => {
      const card = renderMotorCard(motor);
      grid.appendChild(card);
    });
  } catch (error) {
    console.error(error);
    grid.textContent = `Error: ${error.message}`;
  }
}

document.addEventListener("DOMContentLoaded", bootstrap);
