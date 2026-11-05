from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt
from PIL import Image

# Get the exact directory where this script file is saved
SCRIPT_DIR = Path(__file__).parent


def combine_and_label_split_pngs(
    image_paths,
    titles=None,
    x_labels=None,
    y_labels=None,
    cols=4,  # 4 columns since we have 4 total subplots
    figsize=(22, 5),
):
  """Takes PNG images containing two side-by-side plots, splits each into

  separate left and right panels sequentially, and creates a clean grid with
  individual titles and axis labels for all 4 plots.
  """
  sub_images = []

  # 1. Load each image and split it down the middle into left and right halves
  for img_name in image_paths:
    img_path = SCRIPT_DIR / img_name
    try:
      img = Image.open(img_path)
      width, height = img.size

      # Crop left and right halves
      left_img = img.crop((0, 0, width // 2, height))
      right_img = img.crop((width // 2, 0, width, height))

      # Append as a sequential pair: Left panel first, then Right panel
      sub_images.append(("loaded", left_img, img_name))
      sub_images.append(("loaded", right_img, img_name))
    except FileNotFoundError:
      sub_images.append(("error", img_path, img_name))
      sub_images.append(("error", img_path, img_name))

  num_plots = len(sub_images)

  # Fallbacks for labels (must match the total number of subplots = 4)
  if titles is None:
    titles = [f"Plot {i+1}" for i in range(num_plots)]
  if x_labels is None:
    x_labels = ["X Axis" for _ in range(num_plots)]
  if y_labels is None:
    y_labels = ["Y Axis" for _ in range(num_plots)]

  rows = (num_plots + cols - 1) // cols
  fig, axes = plt.subplots(rows, cols, figsize=figsize)

  if num_plots == 1:
    axes = [axes]
  else:
    axes = axes.flatten()

  for i, (status, content, orig_name) in enumerate(sub_images):
    ax = axes[i]
    if status == "loaded":
      ax.imshow(content)
      ax.set_title(titles[i], fontsize=11, fontweight="bold", pad=8)
      ax.set_xlabel(x_labels[i], fontsize=9, fontweight="semibold")
      ax.set_ylabel(y_labels[i], fontsize=9, fontweight="semibold")

      # Hide raw pixel ticks
      ax.set_xticks([])
      ax.set_yticks([])
    else:
      ax.text(
          0.5,
          0.5,
          f"Image not found:\n{orig_name}",
          ha="center",
          va="center",
          fontsize=9,
          color="red",
      )
      ax.axis("off")

  # Hide any empty grid slots
  for j in range(num_plots, len(axes)):
    axes[j].axis("off")

  plt.tight_layout()

  # 2. Save the final combined image in the script's directory
  timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
  output_filename = SCRIPT_DIR / f"combined_plot_{timestamp}.png"

  plt.savefig(output_filename, dpi=300, bbox_inches="tight")
  print(f"Successfully created and saved: {output_filename}")
  plt.close()


# ==========================================
# Example Usage
# ==========================================
if __name__ == "__main__":
  my_pngs = [
      "linear_plot_Point_P_Control_klp=0.1, kap=0.7)=.png",
      "angular_plot_Point_P_Control_klp=0.1, kap=0.7)=.png",
  ]

  # Order matches:
  # [Linear Left (State Space), Linear Right (Individual States),
  #  Angular Left (State Space), Angular Right (Individual States)]
  my_titles = [
      "Linear: State Space",
      "Linear: Individual States",
      "Angular: State Space",
      "Angular: Individual States",
  ]

  my_x_labels = [
      "State X Position",
      "Time (s)",
      "State X Position",
      "Time (s)",
  ]

  my_y_labels = [
      "State Y Position",
      "Error Value",
      "State Y Position",
      "Error Value",
  ]

  combine_and_label_split_pngs(
      image_paths=my_pngs,
      titles=my_titles,
      x_labels=my_x_labels,
      y_labels=my_y_labels,
      cols=4,  # Arranges all 4 panels side by side in a row
      figsize=(22, 5),
  )