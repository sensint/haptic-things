"""
PCA Analysis with Comprehensive Similarity Assessment for Motion Data

This module provides comprehensive analysis of motion data using PCA and multiple similarity assessment methods.

SIMILARITY ASSESSMENT METHODS:

1. Centroid-based Similarity:
   - Computes the centroid (mean) of each file's data points in PCA space
   - Uses cosine similarity between centroids
   - Higher values indicate more similar files
   - Good for overall motion pattern comparison

2. Distribution-based Similarity:
   - Wasserstein Distance: Measures the "work" needed to transform one distribution into another
   - Kolmogorov-Smirnov Test: Statistical test for comparing distributions
   - Lower Wasserstein distances and higher KS p-values indicate more similar distributions
   - Good for comparing the statistical properties of motion patterns

3. Trajectory-based Similarity:
   - Dynamic Time Warping (DTW): Measures similarity between time series trajectories
   - Accounts for temporal alignment and warping
   - Lower DTW distances indicate more similar trajectory patterns
   - Good for comparing the temporal evolution of motion

USAGE:
    python pca_analysis.py

OUTPUTS:
    - PCA results and visualizations
    - Similarity matrices (CSV files)
    - Similarity heatmaps (PNG files)
    - Comprehensive similarity analysis report
"""

import os
import numpy as np
import pandas as pd
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
from sklearn.metrics.pairwise import euclidean_distances, cosine_similarity
from scipy.spatial.distance import cdist
from scipy.stats import wasserstein_distance, ks_2samp
import matplotlib.pyplot as plt
import seaborn as sns
import glob
import sys
from tqdm import tqdm

# Add the src directory to the path to import motion_analysis
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from motion_features import load_motion_data, compute_motion_features

def load_all_motion_data(data_dir='../cube-all'):
    """
    Load all TSV files from the specified directory, compute motion features,
    normalize the data, and combine them with file labels.
    """
    # Find all TSV files in the directory
    tsv_files = glob.glob(os.path.join(data_dir, '*.tsv'))
    # tsv_files = [os.path.join(data_dir, 'cube-x_6D.tsv'), os.path.join(data_dir, 'cube-y_6D.tsv'), os.path.join(data_dir, 'cube-z_6D.tsv')]
    
    if not tsv_files:
        raise FileNotFoundError(f"No TSV files found in {data_dir}")
    
    print(f"Found {len(tsv_files)} TSV files:")
    for file in tsv_files:
        print(f"  - {os.path.basename(file)}")
    
    combined_data = []
    file_labels = []
    
    for file_path in tqdm(tsv_files):
        print(f"\nProcessing {os.path.basename(file_path)}...")
        
        # Load motion data
        df = load_motion_data(file_path)
      
        df = compute_motion_features(df)
        
        df = df.dropna()
        # Extract filename without extension as label
        file_label = os.path.basename(file_path).replace('.tsv', '')
        
        # Add file label column
        df['file_label'] = file_label
        
        # Store the data
        combined_data.append(df)
        file_labels.extend([file_label] * len(df))
        
        print(f"  Loaded {len(df)} rows from {file_label}")
    
    # Combine all dataframes
    combined_df = pd.concat(combined_data, ignore_index=True)
    
    print(f"\nCombined dataset shape: {combined_df.shape}")
    print(f"Unique file labels: {combined_df['file_label'].unique()}")
    
    return combined_df

def prepare_features_for_pca(df, exclude_columns=None):
    """
    Prepare features for PCA by selecting relevant columns and handling categorical data.
    """
    if exclude_columns is None:
        exclude_columns = ['Time', 'file_label', 'Frame', 'motion_type']
    
    # Select only numeric columns for PCA
    feature_columns = df.select_dtypes(include=[np.number]).columns.tolist()
    
    # Remove excluded columns
    feature_columns = [col for col in feature_columns if col not in exclude_columns]
    
    print(f"Selected {len(feature_columns)} features for PCA:")
    for col in feature_columns:
        print(f"  - {col}")
    
    return df[feature_columns], feature_columns

def normalize_data(data):
    """Normalize motion data by standardizing each feature"""
    scaler = StandardScaler()
    normalized_data = scaler.fit_transform(data)
    return scaler, normalized_data

def apply_pca(data, n_components=2):
    """Apply PCA to reduce dimensionality"""
    pca = PCA(n_components=n_components)
    transformed_data = pca.fit_transform(data)
    return pca, transformed_data

def visualize_pca(transformed_data, labels, file_labels, n_components=2):
    """Visualize PCA components"""
    # Convert file_labels to numpy array if it's a pandas Series
    if hasattr(file_labels, 'values'):
        file_labels = file_labels.values
    
    if n_components == 2:
        # 2D scatter plot
        plt.figure(figsize=(12, 8))
        
        # Create scatter plot with different colors for each file
        unique_labels = np.unique(file_labels)
        colors = plt.cm.Set1(np.linspace(0, 1, len(unique_labels)))
        
        for i, label in enumerate(unique_labels):
            mask = file_labels == label
            plt.scatter(transformed_data[mask, 0], transformed_data[mask, 1], 
                       c=[colors[i]], label=label, alpha=0.7, s=20)
        
        plt.xlabel('Principal Component 1')
        plt.ylabel('Principal Component 2')
        plt.title('PCA of Normalized Motion Data (2D)')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        plt.show()
        
    elif n_components >= 3:
        # 3D scatter plot
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Create scatter plot with different colors for each file
        unique_labels = np.unique(file_labels)
        colors = plt.cm.Set1(np.linspace(0, 1, len(unique_labels)))
        
        for i, label in enumerate(unique_labels):
            mask = file_labels == label
            ax.scatter(transformed_data[mask, 0], transformed_data[mask, 1], transformed_data[mask, 2], 
                      c=[colors[i]], label=label, alpha=0.7, s=20)
        
        ax.set_xlabel('Principal Component 1')
        ax.set_ylabel('Principal Component 2')
        ax.set_zlabel('Principal Component 3')
        ax.set_title('PCA of Normalized Motion Data (3D)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        plt.show()

def analyze_pca_results(pca, feature_columns, n_components=2):
    """Analyze and print PCA results"""
    print(f"\nPCA Results (n_components={n_components}):")
    print(f"Explained variance ratio: {pca.explained_variance_ratio_}")
    print(f"Total explained variance: {sum(pca.explained_variance_ratio_):.3f}")
    
    # Print feature importance for each component
    for i in range(n_components):
        print(f"\nPrincipal Component {i+1}:")
        component_weights = pca.components_[i]
        feature_importance = list(zip(feature_columns, component_weights))
        feature_importance.sort(key=lambda x: abs(x[1]), reverse=True)
        
        print("Top 10 most important features:")
        for feature, weight in feature_importance[:10]:
            print(f"  {feature}: {weight:.3f}")

def compute_centroid_similarity(transformed_data, file_labels, n_components=3):
    """
    Compute similarity between files based on their centroids in PCA space.
    Returns a similarity matrix where higher values indicate more similar files.
    """
    # Convert file_labels to numpy array if it's a pandas Series
    if hasattr(file_labels, 'values'):
        file_labels = file_labels.values
    
    unique_labels = np.unique(file_labels)
    n_files = len(unique_labels)
    similarity_matrix = np.zeros((n_files, n_files))
    
    # Compute centroids for each file
    centroids = {}
    for label in unique_labels:
        mask = file_labels == label
        centroids[label] = np.mean(transformed_data[mask, :n_components], axis=0)
    
    # Compute pairwise similarities
    for i, label1 in enumerate(unique_labels):
        for j, label2 in enumerate(unique_labels):
            if i == j:
                similarity_matrix[i, j] = 1.0  # Self-similarity
            else:
                # Use cosine similarity for centroid comparison
                centroid1 = centroids[label1]
                centroid2 = centroids[label2]
                similarity = cosine_similarity([centroid1], [centroid2])[0, 0]
                similarity_matrix[i, j] = similarity
    
    return similarity_matrix, unique_labels, centroids

def compute_distribution_similarity(transformed_data, file_labels, n_components=3):
    """
    Compute similarity between files based on the distribution of their PCA components.
    Uses Wasserstein distance and Kolmogorov-Smirnov test.
    """
    # Convert file_labels to numpy array if it's a pandas Series
    if hasattr(file_labels, 'values'):
        file_labels = file_labels.values
    
    unique_labels = np.unique(file_labels)
    n_files = len(unique_labels)
    wasserstein_matrix = np.zeros((n_files, n_files))
    ks_matrix = np.zeros((n_files, n_files))
    
    for i, label1 in enumerate(unique_labels):
        for j, label2 in enumerate(unique_labels):
            if i == j:
                wasserstein_matrix[i, j] = 0.0
                ks_matrix[i, j] = 1.0
            else:
                mask1 = file_labels == label1
                mask2 = file_labels == label2
                
                # Compute Wasserstein distance for each component
                wasserstein_distances = []
                ks_pvalues = []
                
                for comp in range(n_components):
                    data1 = transformed_data[mask1, comp]
                    data2 = transformed_data[mask2, comp]
                    
                    # Wasserstein distance
                    w_dist = wasserstein_distance(data1, data2)
                    wasserstein_distances.append(w_dist)
                    
                    # Kolmogorov-Smirnov test
                    ks_stat, ks_pval = ks_2samp(data1, data2)
                    ks_pvalues.append(ks_pval)
                
                # Average across components
                wasserstein_matrix[i, j] = np.mean(wasserstein_distances)
                ks_matrix[i, j] = np.mean(ks_pvalues)
    
    return wasserstein_matrix, ks_matrix, unique_labels

def compute_trajectory_similarity(transformed_data, file_labels, n_components=3):
    """
    Compute similarity between files based on their trajectory patterns in PCA space.
    Uses Dynamic Time Warping (DTW) distance.
    """
    try:
        from dtaidistance import dtw
    except ImportError:
        print("Warning: dtaidistance not available. Skipping trajectory similarity.")
        return None, None, None
    
    # Convert file_labels to numpy array if it's a pandas Series
    if hasattr(file_labels, 'values'):
        file_labels = file_labels.values
    
    unique_labels = np.unique(file_labels)
    n_files = len(unique_labels)
    dtw_matrix = np.zeros((n_files, n_files))
    
    for i, label1 in enumerate(unique_labels):
        for j, label2 in enumerate(unique_labels):
            if i == j:
                dtw_matrix[i, j] = 0.0
            else:
                mask1 = file_labels == label1
                mask2 = file_labels == label2
                
                # Get trajectories for each file
                traj1 = transformed_data[mask1, :n_components]
                traj2 = transformed_data[mask2, :n_components]
                
                # Compute DTW distance
                distance = dtw.distance(traj1, traj2)
                dtw_matrix[i, j] = distance
    
    return dtw_matrix, unique_labels

def visualize_similarity_matrices(similarity_matrices, labels, save_path=None):
    """
    Visualize similarity matrices using heatmaps with diagonal from lower left to top right.
    """
    n_matrices = len(similarity_matrices)
    fig, axes = plt.subplots(1, n_matrices, figsize=(5*n_matrices, 4))
    
    if n_matrices == 1:
        axes = [axes]
    
    # Create feedback labels (feedback-a, feedback-b, etc.)
    feedback_labels = [f'feedback-{chr(97+i)}' for i in range(len(labels))]  # 97 is ASCII for 'a'
    
    for i, (name, matrix) in enumerate(similarity_matrices.items()):
        # Rotate the matrix to show diagonal from lower left to top right
        # This flips the matrix both horizontally and vertically
        # rotated_matrix = np.flipud(np.fliplr(matrix))
        matrix = matrix.T
        
        # Create a mask for values that are 1.0 (self-similarity)
        mask = np.isclose(matrix, 1.0, atol=1e-6)
        
        # Create heatmap with rotated matrix and feedback labels
        sns.heatmap(matrix, annot=True, fmt='.3f', cmap='viridis', 
                   xticklabels=feedback_labels,  # Reverse labels for rotated view
                   yticklabels=feedback_labels, 
                #    mask=mask,  # Hide values that are 1.0
                mask=mask,
                # annot=True,
                   ax=axes[i])
        
        # plt.gca().invert_yaxis()
        axes[i].invert_yaxis()
        axes[i].set_title(f'{name} Similarity Matrix')
        axes[i].set_xlabel('Feedback Conditions')
        axes[i].set_ylabel('Feedback Conditions')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Similarity matrices saved to: {save_path}")
    plt.show()

def compute_comprehensive_similarity(transformed_data, file_labels, n_components=3):
    """
    Compute comprehensive similarity assessment using multiple methods.
    """
    print("\nComputing similarity assessments...")
    
    # 1. Centroid-based similarity
    print("Computing centroid-based similarity...")
    centroid_sim, labels, centroids = compute_centroid_similarity(transformed_data, file_labels, n_components)
    
    # 2. Distribution-based similarity
    print("Computing distribution-based similarity...")
    wasserstein_sim, ks_sim, _ = compute_distribution_similarity(transformed_data, file_labels, n_components)
    
    # # 3. Trajectory-based similarity
    # print("Computing trajectory-based similarity...")
    # dtw_sim, _ = compute_trajectory_similarity(transformed_data, file_labels, n_components)
    
    # Create similarity matrices dictionary
    similarity_matrices = {
        'Centroid (Cosine)': centroid_sim,
        # 'Distribution (Wasserstein)': wasserstein_sim,
        # 'Distribution (KS Test)': ks_sim
    }
    
    # if dtw_sim is not None:
    #     similarity_matrices['Trajectory (DTW)'] = dtw_sim
    
    return similarity_matrices, labels, centroids

def analyze_file_similarities(similarity_matrices, labels):
    """
    Analyze and print detailed similarity information between files.
    """
    print("\n" + "="*60)
    print("SIMILARITY ANALYSIS RESULTS")
    print("="*60)
    
    # Create feedback labels for display
    feedback_labels = [f'feedback-{chr(97+i)}' for i in range(len(labels))]
    label_mapping = dict(zip(labels, feedback_labels))
    
    for method_name, matrix in similarity_matrices.items():
        print(f"\n{method_name.upper()} SIMILARITY:")
        print("-" * 40)
        
        # Find most similar and least similar pairs
        n_files = len(labels)
        similarities = []
        
        for i in range(n_files):
            for j in range(i+1, n_files):
                sim_value = matrix[i, j]
                similarities.append((labels[i], labels[j], sim_value))
        
        # Sort by similarity (descending for cosine/ks, ascending for distances)
        if 'Wasserstein' in method_name or 'DTW' in method_name:
            similarities.sort(key=lambda x: x[2])  # Ascending for distances
            print("Most similar pairs (lowest distance):")
            for i, (file1, file2, sim) in enumerate(similarities[:3]):
                feedback1 = label_mapping[file1]
                feedback2 = label_mapping[file2]
                print(f"  {feedback1} ↔ {feedback2}: {sim:.3f}")
            
            print("\nLeast similar pairs (highest distance):")
            for i, (file1, file2, sim) in enumerate(similarities[-3:]):
                feedback1 = label_mapping[file1]
                feedback2 = label_mapping[file2]
                print(f"  {feedback1} ↔ {feedback2}: {sim:.3f}")
        else:
            similarities.sort(key=lambda x: x[2], reverse=True)  # Descending for similarities
            print("Most similar pairs:")
            for i, (file1, file2, sim) in enumerate(similarities[:3]):
                feedback1 = label_mapping[file1]
                feedback2 = label_mapping[file2]
                print(f"  {feedback1} ↔ {feedback2}: {sim:.3f}")
            
            print("\nLeast similar pairs:")
            for i, (file1, file2, sim) in enumerate(similarities[-3:]):
                feedback1 = label_mapping[file1]
                feedback2 = label_mapping[file2]
                print(f"  {feedback1} ↔ {feedback2}: {sim:.3f}")
        
        # Print average similarity
        avg_sim = np.mean([sim for _, _, sim in similarities])
        print(f"\nAverage similarity: {avg_sim:.3f}")

def save_similarity_results(similarity_matrices, labels, output_dir='../cube-all'):
    """
    Save similarity results to CSV files.
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Create feedback labels for saving
    feedback_labels = [f'feedback-{chr(97+i)}' for i in range(len(labels))]
    
    for method_name, matrix in similarity_matrices.items():
        # Create DataFrame with feedback labels
        df = pd.DataFrame(matrix, index=feedback_labels, columns=feedback_labels)
        
        # Save to CSV
        filename = f"{method_name.lower().replace(' ', '_').replace('(', '').replace(')', '')}_similarity.csv"
        filepath = os.path.join(output_dir, filename)
        df.to_csv(filepath)
        print(f"Saved {method_name} similarity matrix to: {filepath}")
    
    # Save combined results
    combined_results = {}
    for method_name, matrix in similarity_matrices.items():
        for i, label1 in enumerate(labels):
            for j, label2 in enumerate(labels):
                if i < j:  # Only upper triangle to avoid duplicates
                    feedback1 = feedback_labels[i]
                    feedback2 = feedback_labels[j]
                    key = f"{feedback1}_vs_{feedback2}"
                    if key not in combined_results:
                        combined_results[key] = {}
                    combined_results[key][method_name] = matrix[i, j]
    
    combined_df = pd.DataFrame(combined_results).T
    combined_filepath = os.path.join(output_dir, 'combined_similarity_results.csv')
    combined_df.to_csv(combined_filepath)
    print(f"Saved combined similarity results to: {combined_filepath}")

def main():
    """Main function to run the complete PCA analysis pipeline"""
    print("Starting PCA analysis of motion data...")
    
    # Load all motion data
    combined_df = load_all_motion_data()
    
    # Prepare features for PCA
    features_df, feature_columns = prepare_features_for_pca(combined_df)
    
    # Normalize the data
    print("\nNormalizing data...")
    scaler, normalized_data = normalize_data(features_df)
    
    # Apply PCA
    print("\nApplying PCA...")
    n_components = 5  # You can change this to 2 for 2D visualization
    pca, transformed_data = apply_pca(normalized_data, n_components=n_components)
    
    # Analyze results
    analyze_pca_results(pca, feature_columns, n_components)
    
    # Visualize results
    print("\nCreating visualizations...")
    visualize_pca(transformed_data, None, combined_df['file_label'].values, n_components)
    
    # Save results
    results_df = pd.DataFrame(transformed_data, columns=[f'PC{i+1}' for i in range(n_components)])
    results_df['file_label'] = combined_df['file_label'].values
    
    output_path = '../cube-all/pca_results.csv'
    results_df.to_csv(output_path, index=False)
    print(f"\nPCA results saved to: {output_path}")
    
    # Compute similarity assessments
    print("\nComputing similarity assessments...")
    similarity_matrices, labels, centroids = compute_comprehensive_similarity(transformed_data, combined_df['file_label'].values, n_components)
    
    # Analyze and visualize similarity matrices
    visualize_similarity_matrices(similarity_matrices, labels, save_path='../cube-all/similarity_matrices.png')
    
    # Analyze file similarities
    analyze_file_similarities(similarity_matrices, labels)
    
    # Save similarity results
    save_similarity_results(similarity_matrices, labels)
    
    print("\nPCA analysis completed!")

if __name__ == "__main__":
    main()
