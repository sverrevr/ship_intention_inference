// This file interfaces the smile library. It implements commanly used functions.
// This file should not care what the BBN is used for.

#pragma once
#include "smile/smile.h"
#include "smile/smile_license.h"
#include <assert.h>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <iostream>
#include "geometry.h"
#include <stdio.h>

namespace INTENTION_INFERENCE
{
class BayesianNetwork{
    private:
    DSL_network net;

    auto getNodeId(std::string name) const{
        const auto node_id = net.FindNode(name.c_str());
        if(node_id<0) printf("ERROR: Node name \"%s\" resulted in error", name.c_str());
        assert(node_id>=0);
        return node_id;
    }

    auto getOutcomeId(std::string node_name, std::string outcome_name){
        const auto node_id = getNodeId(node_name);
        const auto outcome_id = net.GetNode(node_id)->Definition()->GetOutcomesNames()->FindPosition(outcome_name.c_str());
        if(outcome_id<0)printf("ERROR: Outcome name \"%s\" resulted in error for node name \"%s\"", outcome_name.c_str(), node_name.c_str());
        assert(outcome_id>=0);
        return outcome_id;
    }

    auto getNumberOfOutcomes(std::string node_name){
        const auto node_id = getNodeId(node_name);
        const auto outcome_count = net.GetNode(node_id)->Definition()->GetNumberOfOutcomes();
        if(outcome_count<=0) printf("ERROR: No outcomes for node_id %s", node_name.c_str());
        assert(outcome_count>0);
        return outcome_count;
    }

    void setDefinition(std::string node_name, DSL_doubleArray& CPT){
        double sum=0;
        for(int i=0; i<CPT.GetSize(); ++i){
            sum += CPT[i];
        }
        if(sum<0.9999 || sum>1.00001 || !isfinite(sum)) printf("ERROR: Prior distribution on \"%s\" sums to %f, should be 1", node_name.c_str(), sum);
        assert(sum>=0.9999 && sum<=1.00001);
        const auto node_id = getNodeId(node_name);
        auto result =  net.GetNode(node_id)->Definition()->SetDefinition(CPT);
        if(result<0) printf("ERROR: Setting priors failed on node \"%s\"", node_name.c_str());
        assert(result>=0);
    }

    void wrapTimeSlice(int* time_slice){
        const auto num_slices = net.GetNumberOfSlices();
        if (*time_slice < 0){
            *time_slice = num_slices+*time_slice;
        }
        if(*time_slice>=num_slices) printf("ERROR: Attempted to access nonexisting timeslice %d", *time_slice);
        assert(*time_slice<num_slices);
    }

    bool isTemporal(int node_id){
        return net.GetTemporalType(node_id) == dsl_temporalType::dsl_plateNode;
    }

    double getTemporalOutcome(int node_id,int time_slice, int outcome_id){
        wrapTimeSlice(&time_slice);
        const auto outcome_count = net.GetNode(node_id)->Definition()->GetNumberOfOutcomes();
        int index = time_slice*outcome_count+outcome_id;
        //Temporal data is saved as a large vector the outcome for each timestep saved after each other
        const auto outcomes = net.GetNode(node_id)->Value()->GetMatrix();
        return (*outcomes)[index];
    }

    auto getTemporalOutcome(std::string node_name,int time_slice, std::string outcome){
        const auto node_id = getNodeId(node_name);
        const auto outcome_id = getOutcomeId(node_name, outcome);
        return getTemporalOutcome(node_id,time_slice,outcome_id);
    }

    double getOutcome(int node_id, int outcome_id){
        return net.GetNode(node_id)->Value()->GetMatrix()->operator[](outcome_id);
    }

    auto getOutcome(std::string node_name,int time_slice=-1){
        std::map<std::string,double> return_value;
        const auto node_id = getNodeId(node_name);
        const auto is_temporal = isTemporal(node_id);
        const auto outcome_count = getNumberOfOutcomes(node_name);
        const auto outcome_names = net.GetNode(node_id)->Definition()->GetOutcomesNames();
        for(int i=0; i<outcome_count;++i){
            if(is_temporal)
                return_value[(*outcome_names)[i]] = getTemporalOutcome(node_id,time_slice,i);
            else
                return_value[(*outcome_names)[i]] = getOutcome(node_id,i);
        }
        return return_value;
    }

public:
    BayesianNetwork(){}

    BayesianNetwork(std::string file_name, unsigned num_network_evaluation_samples){
        init(file_name, num_network_evaluation_samples);
    }

    void init(std::string file_name, unsigned num_network_evaluation_samples){
        DSL_errorH().RedirectToFile(stdout); //Rederects errors to standard output
        std::string full_path = ros::package::getPath("intent_inference")+file_name;
        const auto result = net.ReadFile(full_path.c_str());
        if(result<0) printf("ERROR: Unable to read file: \"%s\"", full_path.c_str());
        assert(result>=0);
        const auto result_slice = net.SetNumberOfSlices(1);
        if(result_slice<0) printf("ERROR: Insufficient number of time slices");
        assert(result_slice>=0);
        net.SetDefaultBNAlgorithm(DSL_ALG_BN_EPISSAMPLING);
        const auto return_set_samples = net.SetNumberOfSamples(num_network_evaluation_samples);
        if(return_set_samples<0) printf("ERROR: Illigal number of samples set: %d", num_network_evaluation_samples);
        assert(return_set_samples>=0);
    }

    void save_network(std::string file_name){
        std::string full_path = ros::package::getPath("intent_inference")+"/include/"+file_name+".xdsl";
        const auto result = net.WriteFile(full_path.c_str());
        if(result<0) printf("ERROR: Unable to write file: \"%s\"", full_path.c_str());
        assert(result>=0);
        printf("Network saved to %s", full_path.c_str());
    }

    void setEvidence(std::string node_name, int outcome_id,int time_slice=-1){
        wrapTimeSlice(&time_slice);
        const auto node_id = getNodeId(node_name);
        if(isTemporal(node_id)){
            const auto res = net.GetNode(node_id)->Value()->SetTemporalEvidence(time_slice,outcome_id);
            if(res<0) printf("ERROR: Set temporal evidence (t=%d, outcome_id=%d) on node \"%s\" resulted in error",time_slice, outcome_id, node_name.c_str());
            assert(res>=0);
        }
        else{
            const auto res = net.GetNode(node_id)->Value()->SetEvidence(outcome_id);
            if(res<0) printf("ERROR: Set evidence (outcome_id=%d) on node \"%s\" resulted in error",outcome_id,node_name.c_str());
            assert(res>=0);
        }
    }

    void setEvidence(std::string node_name, std::string observed_outcome,int time_slice=-1){
        const auto outcome_id = getOutcomeId(node_name, observed_outcome);
        setEvidence(node_name, outcome_id,time_slice);
    }

    //Input <node_name, observation>
    void setEvidence(const std::map<std::string,std::string>&  evidence){
        for(auto e : evidence){
            setEvidence(e.first,e.second);
        }
    }

    void setPriors(std::string node_name, const std::map<std::string, double>& prior_distribution){
        const auto node_id = getNodeId(node_name);
        auto node_definition = net.GetNode(node_id)->Definition();
        DSL_doubleArray CPT(node_definition->GetMatrix()->GetSize());
        for(const auto& [outcome_name, outcome_prob] : prior_distribution){
            const auto outcome_id = getOutcomeId(node_name, outcome_name);
            CPT[outcome_id] = outcome_prob;
        }
        setDefinition(node_name, CPT);
    }

    void setBinaryPriors(std::string node_name, double probablity_of_true){
        setPriors(node_name, {{"false", 1-probablity_of_true},{"true", probablity_of_true}});
    }

    void setPriorNormalDistribution(const std::string node_name, const double mu, const double sigma, const double bin_width){
        const auto node_id = getNodeId(node_name);
        auto node_definition = net.GetNode(node_id)->Definition();
        DSL_doubleArray CPT(node_definition->GetMatrix()->GetSize());
        CPT[0] = evaluateBinProbability(-INFINITY, bin_width, mu, sigma); //First bin takes everything below 0 aswell
        for(auto i=1; i<CPT.GetSize()-1; ++i){
            CPT[i] = evaluateBinProbability(i*bin_width, (i+1)*bin_width, mu, sigma);
        }
        CPT[CPT.GetSize()-1] = evaluateBinProbability((CPT.GetSize()-1)*bin_width, INFINITY, mu, sigma); //Last bin takes everything above max
        setDefinition(node_name, CPT);
    }

    void setVirtualEvidence(std::string node_name,const std::vector<double>& virtualEvidence, int time_slice=-1){
        wrapTimeSlice(&time_slice);
        const auto node_id = getNodeId(node_name);
        if(isTemporal(node_id)){
            const auto result = net.GetNode(node_id)->Value()->SetTemporalEvidence(time_slice,virtualEvidence);
            if(result<0) printf("ERROR: Set virtual evidence (t=%d) on node \"%s\" resulted in error", time_slice, node_name.c_str());
            assert(result>=0);
        }
        else{
            const auto result = net.GetNode(node_id)->Value()->SetVirtualEvidence(virtualEvidence);
            if(result<0) printf("ERROR: Set virtual evidence on node \"%s\" resulted in error", node_name.c_str());
            assert(result>=0);
        }
    }

    void setVirtualEvidence(std::string node_name, const std::map<std::string, double>& virtualEvidence){
        const auto num_outcomes = getNumberOfOutcomes(node_name);
        assert(num_outcomes>0);
        std::vector<double> virtual_evidence_vector(num_outcomes, 0.0);
        for(auto evidence:virtualEvidence){
            const auto outcome_id = getOutcomeId(node_name, evidence.first);
            virtual_evidence_vector[outcome_id] = evidence.second;
        }
        setVirtualEvidence(node_name,virtual_evidence_vector);
    }

    void incrementTime(){
        const auto number_of_time_slices = net.GetNumberOfSlices();
        const auto res = net.SetNumberOfSlices(number_of_time_slices+1);
        if(res<0) printf("ERROR: Increment time failed");
        assert(res>=0);
    }

    void decrementTime(){
        const auto number_of_time_slices = net.GetNumberOfSlices();
        const auto res = net.SetNumberOfSlices(number_of_time_slices-1);
        if(res<0) printf("ERROR: Decrement time failed");
        assert(res>=0);
    }

    auto evaluateStates(std::vector<std::string> node_names){
        const auto current_time_slice = net.GetNumberOfSlices()-1;

        //Target nodes are the nodes the inference algorithm tries to solve.
        for(auto node_name:node_names){
            net.SetTarget(getNodeId(node_name));
        }
        const auto update_res = net.UpdateBeliefs();
        if(update_res<0){
            printf("ERROR: Unable to update beliefs");
            throw "Unable to update beliefs";
        } 

        assert(update_res>=0);

        std::map<std::string,std::map<std::string,double>> return_value;
        for(auto node_name:node_names){
            return_value[node_name] = getOutcome(node_name,current_time_slice);
        }
        return return_value;
    }

    auto getNumberOfTimeSteps()const{
        return net.GetNumberOfSlices();
    }
};
}
